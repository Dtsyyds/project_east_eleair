"""Explicit FP32 CPU backend option; tensor shape and normalization unchanged."""
import torch

class CPUInference:
    def __init__(self, model, backend='cpu'):
        if backend not in ('cpu', 'cpu_channels_last'):
            raise ValueError('unsupported inference backend')
        self.model=model
        self.backend=backend
        if backend=='cpu_channels_last':
            self.model.to(memory_format=torch.channels_last)

    def probability(self, nchw):
        with torch.inference_mode():
            tensor=torch.from_numpy(nchw)
            if self.backend=='cpu_channels_last':
                tensor=tensor.contiguous(memory_format=torch.channels_last)
            return torch.sigmoid(self.model(tensor)).numpy()[0,0]
