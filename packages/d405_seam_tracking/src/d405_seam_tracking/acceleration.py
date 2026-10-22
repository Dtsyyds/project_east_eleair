"""Real local S100 OpenCL GPU and checked native HBM adapter. No simulated backend."""
import ctypes as C
import json
from pathlib import Path
import time
import numpy as np
from .contracts import sha256

KERNEL = r'''
__kernel void wall(__global const float *z, __global const uchar *mask,
                   __global const float *p, __global float4 *out, const int width, const int count) {
  int i=get_global_id(0); if(i>=count) return;
  float Z=z[i];
  if(!mask[i] || !isfinite(Z) || Z<=0) { out[i]=(float4)(NAN,NAN,NAN,NAN); return; }
  float X=((float)(i%width)+p[4]-p[2])*Z/p[0];
  float Y=((float)(i/width)+p[5]-p[3])*Z/p[1];
  float x=(X-p[12])/p[14], y=(Y-p[13])/p[15];
  float predicted=p[6]+p[7]*x+p[8]*y+p[9]*x*x+p[10]*x*y+p[11]*y*y;
  float dx=(p[7]+2*p[9]*x+p[10]*y)/p[14];
  float dy=(p[8]+p[10]*x+2*p[11]*y)/p[15];
  out[i]=(float4)(X,Y,Z,(predicted-Z)/sqrt(1+dx*dx+dy*dy));
}
'''


class OpenCLGeometry:
    def __init__(self, timeout_sec=2.):
        self.timeout_sec, self.buffers, self.size, self.event = timeout_sec, [], 0, None
        self.initialized = False
        self.l = C.CDLL('/usr/lib/aarch64-linux-gnu/libOpenCL.so.1')
        p, u, s, i = C.c_void_p, C.c_uint, C.c_size_t, C.c_int
        specs = {
            'clGetPlatformIDs': (i, [u, C.POINTER(p), C.POINTER(u)]),
            'clGetDeviceIDs': (i, [p, C.c_ulong, u, C.POINTER(p), C.POINTER(u)]),
            'clGetDeviceInfo': (i, [p, u, s, p, C.POINTER(s)]),
            'clCreateContext': (p, [p, u, C.POINTER(p), p, p, C.POINTER(i)]),
            'clCreateCommandQueue': (p, [p, p, C.c_ulong, C.POINTER(i)]),
            'clCreateProgramWithSource': (p, [p, u, C.POINTER(C.c_char_p), C.POINTER(s), C.POINTER(i)]),
            'clBuildProgram': (i, [p, u, C.POINTER(p), C.c_char_p, p, p]),
            'clGetProgramBuildInfo': (i, [p, p, u, s, p, C.POINTER(s)]),
            'clCreateKernel': (p, [p, C.c_char_p, C.POINTER(i)]),
            'clCreateBuffer': (p, [p, C.c_ulong, s, p, C.POINTER(i)]),
            'clSetKernelArg': (i, [p, u, s, p]),
            'clEnqueueWriteBuffer': (i, [p, p, u, s, s, p, u, p, C.POINTER(p)]),
            'clEnqueueReadBuffer': (i, [p, p, u, s, s, p, u, p, C.POINTER(p)]),
            'clEnqueueNDRangeKernel': (i, [p, p, u, p, C.POINTER(s), p, u, p, C.POINTER(p)]),
            'clFlush': (i, [p]), 'clGetEventInfo': (i, [p, u, s, p, C.POINTER(s)])}
        for name in ('clReleaseMemObject', 'clReleaseKernel', 'clReleaseProgram', 'clReleaseCommandQueue', 'clReleaseContext', 'clReleaseEvent'):
            specs[name] = (i, [p])
        for name, (restype, args) in specs.items():
            f = getattr(self.l, name); f.restype, f.argtypes = restype, args
        n = u(); self.check(self.l.clGetPlatformIDs(0, None, C.byref(n)))
        platforms = (p * n.value)(); self.check(self.l.clGetPlatformIDs(n, platforms, None))
        self.device = p()
        for platform in platforms:
            # CL_DEVICE_TYPE_GPU only. A CPU implementation is NOT accepted as GPU.
            if self.l.clGetDeviceIDs(platform, 4, 1, C.byref(self.device), None) == 0: break
        else: raise RuntimeError('opencl_gpu_unavailable')
        name = C.create_string_buffer(512)
        self.check(self.l.clGetDeviceInfo(self.device, 0x102B, 512, name, None))
        self.device_name = name.value.decode()
        if not any(x in self.device_name.lower() for x in ('mali-g78ae',)):
            raise RuntimeError('not_verified_s100_gpu:' + self.device_name)
        error = i()
        self.context = self.l.clCreateContext(None, 1, C.byref(self.device), None, None, C.byref(error)); self.check(error.value)
        self.queue = self.l.clCreateCommandQueue(self.context, self.device, 0, C.byref(error)); self.check(error.value)
        source = C.c_char_p(KERNEL.encode())
        self.program = self.l.clCreateProgramWithSource(self.context, 1, C.byref(source), None, C.byref(error)); self.check(error.value)
        code = self.l.clBuildProgram(self.program, 1, C.byref(self.device), b'', None, None)
        if code:
            log = C.create_string_buffer(16384)
            self.l.clGetProgramBuildInfo(self.program, self.device, 0x1183, len(log), log, None)
            self.close()
            raise RuntimeError('opencl_build_failed:' + log.value.decode())
        self.kernel = self.l.clCreateKernel(self.program, b'wall', C.byref(error)); self.check(error.value)
        self.initialized = True

    def check(self, code):
        if code:
            if not self.initialized: self.close()
            raise RuntimeError(f'OpenCL error {code}')

    def compute(self, z, mask, calibration, bounds, coefficients, origin, scale):
        if self.event is not None: raise RuntimeError('OpenCL prior task not safely completed')
        start = time.perf_counter()
        z, mask = np.ascontiguousarray(z, dtype=np.float32), np.ascontiguousarray(mask, dtype=np.uint8)
        if z.shape != mask.shape or z.ndim != 2: raise ValueError('GPU shape mismatch')
        coef = np.zeros(6, np.float32); coef[:len(coefficients)] = coefficients
        k = np.array(calibration.k).reshape(3, 3)
        params = np.array([k[0, 0], k[1, 1], k[0, 2], k[1, 2], bounds[0], bounds[1], *coef, *origin, *scale], np.float32)
        output = np.empty(z.shape + (4,), np.float32)
        if self.size != z.size:
            for buffer in self.buffers: self.check(self.l.clReleaseMemObject(buffer))
            self.buffers = []
            for size in (z.nbytes, mask.nbytes, params.nbytes, output.nbytes):
                error = C.c_int(); buffer = self.l.clCreateBuffer(self.context, 1, size, None, C.byref(error)); self.check(error.value)
                self.buffers.append(buffer)
            self.size = z.size
        # Nonblocking transfers and one ordered queue. Host arrays remain alive through completion.
        self.pending_arrays = (z, mask, params, output)
        for buffer, array in zip(self.buffers, (z, mask, params)):
            self.check(self.l.clEnqueueWriteBuffer(self.queue, buffer, 0, 0, array.nbytes, array.ctypes.data, 0, None, None))
        for index, buffer in enumerate(self.buffers):
            handle = C.c_void_p(buffer)
            self.check(self.l.clSetKernelArg(self.kernel, index, C.sizeof(handle), C.byref(handle)))
        for index, value in enumerate((z.shape[1], z.size), 4):
            v = C.c_int(value); self.check(self.l.clSetKernelArg(self.kernel, index, C.sizeof(v), C.byref(v)))
        count = C.c_size_t(z.size)
        self.check(self.l.clEnqueueNDRangeKernel(self.queue, self.kernel, 1, None, C.byref(count), None, 0, None, None))
        self.event = C.c_void_p()
        self.check(self.l.clEnqueueReadBuffer(self.queue, self.buffers[-1], 0, 0, output.nbytes, output.ctypes.data, 0, None, C.byref(self.event)))
        self.check(self.l.clFlush(self.queue))
        deadline = time.monotonic() + self.timeout_sec
        while True:
            status = C.c_int()
            self.check(self.l.clGetEventInfo(self.event, 0x11D3, C.sizeof(status), C.byref(status), None))
            if status.value == 0: break
            if status.value < 0: raise RuntimeError('OpenCL task failed:' + str(status.value))
            if time.monotonic() >= deadline: raise TimeoutError('OpenCL deadline exceeded; task and buffers retained, unsafe to release')
            time.sleep(.0005)
        self.check(self.l.clReleaseEvent(self.event)); self.event = None; self.pending_arrays = None
        self.last_wall_ms = (time.perf_counter() - start) * 1000
        return output

    def residual(self, *args):
        return self.compute(*args)[:, :, 3]

    def close(self):
        if self.event is not None:
            status = C.c_int(); self.check(self.l.clGetEventInfo(self.event, 0x11D3, C.sizeof(status), C.byref(status), None))
            if status.value > 0: raise TimeoutError('OpenCL task still active; resources retained')
            self.check(self.l.clReleaseEvent(self.event)); self.event = None
        for b in self.buffers: self.l.clReleaseMemObject(b)
        self.buffers = []
        for name, attr in [('clReleaseKernel', 'kernel'), ('clReleaseProgram', 'program'), ('clReleaseCommandQueue', 'queue'), ('clReleaseContext', 'context')]:
            handle = getattr(self, attr, None)
            if handle: getattr(self.l, name)(handle); setattr(self, attr, None)


class BPUVisual:
    """Installed S100 native DNN/UCP API owns alignment/cache/task wait/free.

    No NV12 reinterpretation. Nash-m compiler contract and explicit model SHA mandatory.
    Actual operator placement remains external compiler evidence, not inferred from suffix.
    """
    def __init__(self, hbm, contract_path):
        if not hbm or not Path(hbm).is_file(): raise RuntimeError('bpu_unavailable: seam HBM missing')
        if not contract_path: raise RuntimeError('bpu_model_contract_required')
        self.contract = json.loads(Path(contract_path).read_text())
        c = self.contract
        if c['layout'] not in ('NCHW', 'NHWC'): raise ValueError('unknown HBM layout')
        if c['target'] != 'nash-m' or c['hbm_sha256'] != sha256(hbm) or c['preprocessing'] != 'luminance_uint8_div255':
            raise ValueError('HBM hash/target/input mismatch')
        if not c.get('compiler_log') or not c.get('operator_placement'): raise ValueError('compiler_evidence_required')
        if sha256(c['compiler_log']) != c['compiler_log_sha256']: raise ValueError('compiler_log_hash_mismatch')
        library = c.get('native_library')
        if not library or not Path(library).is_file(): raise RuntimeError('build native BPU adapter and set native_library in model contract')
        self.native = C.CDLL(library)
        p = C.c_void_p
        self.native.d405_bpu_open.argtypes = [C.c_char_p]; self.native.d405_bpu_open.restype = p
        self.native.d405_bpu_info.argtypes = [p]; self.native.d405_bpu_info.restype = C.c_char_p
        self.native.d405_bpu_error.restype = C.c_char_p
        self.native.d405_bpu_run.argtypes = [p, p, C.c_size_t, p, C.c_size_t, C.c_int]; self.native.d405_bpu_run.restype = C.c_int
        self.native.d405_bpu_close.argtypes = [p]; self.native.d405_bpu_close.restype = C.c_int
        self.handle = self.native.d405_bpu_open(str(hbm).encode())
        if not self.handle: raise RuntimeError(self.native.d405_bpu_error().decode())
        self.info = json.loads(self.native.d405_bpu_info(self.handle))
        if self.info['input_shape'] != c['input_shape'] or self.info['output_shape'] != c['output_shape']:
            self.close(); raise ValueError('HBM IO shape mismatch')

    def infer(self, image):
        import cv2
        from .learning import preprocess
        shape = self.contract['input_shape']
        size = tuple(shape[2:]) if self.contract['layout'] == 'NCHW' else tuple(shape[1:3])
        x, transform = preprocess(image, size)
        from dataclasses import asdict
        self.last_transform = asdict(transform)
        if self.contract['layout'] == 'NHWC': x = np.transpose(x, (0, 2, 3, 1))
        tick = time.perf_counter()
        x = np.ascontiguousarray(x, dtype=np.float32)
        y = np.empty(self.contract['output_shape'], dtype=np.float32)
        code = self.native.d405_bpu_run(self.handle, x.ctypes.data, x.size, y.ctypes.data, y.size, int(self.contract.get('timeout_ms', 1000)))
        if code: raise RuntimeError(self.native.d405_bpu_error().decode())
        if list(y.shape) != self.contract['output_shape'] or np.prod(y.shape) != size[0] * size[1]: raise ValueError('not binary full-resolution logits')
        self.last_wall_ms = (time.perf_counter() - tick) * 1000
        p = 1 / (1 + np.exp(-np.clip(y.reshape(size), -40, 40)))
        return cv2.resize(p, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_LINEAR)

    def close(self):
        if self.handle:
            if self.native.d405_bpu_close(self.handle): raise TimeoutError(self.native.d405_bpu_error().decode())
            self.handle = None
