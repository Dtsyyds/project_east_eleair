#!/usr/bin/env python3
"""Concatenate prior and relocated PCD maps already expressed in the same coordinates.

Uses only Python's standard library. Preserves every field and point, without
registration, coordinate transformation, downsampling, or duplicate removal.
Supports matching DATA binary or ascii PCD files from FAST_LIVO2_Ultra.
Edit the filename settings below and run without arguments.
"""

import argparse
from contextlib import ExitStack
import os
from pathlib import Path
import shutil
import sys
import tempfile


# ================= 只需修改这里的文件名 =================
# 先验输入固定读取 PCD/priori/，relocate 输入固定读取 PCD/relocate/。
PRIOR_PCD_NAME = "map_1103_2217.pcd"
RELOCATE_PCD_NAME = "map_1104_1821.pcd"

# 合并结果保存到 PCD/merge/，自动命名为 merged_<先验时间>_<relocate时间>.pcd。
# 例如 map_1103_2217.pcd + map_1103_2221.pcd → merged_1103_2217_1103_2221.pcd。
# 已有同名合并结果时，改为 True 允许替换输出；False 会拒绝覆盖。
OVERWRITE_OUTPUT = False
# ======================================================

PCD_DIR = Path(__file__).resolve().parent.parent


def read_header(stream):
    header = {}
    size = 0
    while True:
        line = stream.readline(65537)
        size += len(line)
        if not line or len(line) > 65536 or size > 262144:
            raise ValueError(f"PCD 文件头缺失或过长：{stream.name}")
        try:
            text = line.decode("ascii").strip()
        except UnicodeDecodeError as error:
            raise ValueError(f"PCD 文件头不是 ASCII：{stream.name}") from error
        if not text or text.startswith("#"):
            continue
        key, *values = text.split()
        key = key.upper()
        if key in header:
            raise ValueError(f"PCD 文件头重复字段 {key}：{stream.name}")
        header[key] = values
        if key == "DATA":
            break

    try:
        fields = tuple(header["FIELDS"])
        sizes = tuple(int(v) for v in header["SIZE"])
        types = tuple(header["TYPE"])
        counts = tuple(int(v) for v in header.get("COUNT", ["1"] * len(fields)))
        points = int(header["POINTS"][0])
        width, height = int(header["WIDTH"][0]), int(header["HEIGHT"][0])
        encoding = header["DATA"][0].lower()
    except (KeyError, IndexError, ValueError) as error:
        raise ValueError(f"PCD 文件头字段不完整或数值无效：{stream.name}") from error
    if not fields or len(set(fields)) != len(fields) or not {"x", "y", "z"}.issubset(fields):
        raise ValueError(f"PCD 必须包含不重复的字段及 x/y/z：{stream.name}")
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError(f"FIELDS/SIZE/TYPE/COUNT 长度不一致：{stream.name}")
    if points < 1 or width < 1 or height < 1 or width * height != points:
        raise ValueError(f"PCD 为空或 WIDTH/HEIGHT 与 POINTS 不一致：{stream.name}")
    if any(n < 1 for n in counts) or any(
        t not in ("F", "I", "U") or s not in ((4, 8) if t == "F" else (1, 2, 4, 8))
        for s, t in zip(sizes, types)
    ):
        raise ValueError(f"PCD 字段类型、大小或 COUNT 不支持：{stream.name}")
    if encoding not in ("binary", "ascii"):
        raise ValueError(f"仅支持 DATA binary/ascii，不支持 {encoding}：{stream.name}")
    if encoding == "binary":
        expected = points * sum(s * n for s, n in zip(sizes, counts))
        remaining = os.fstat(stream.fileno()).st_size - stream.tell()
        if remaining != expected:
            raise ValueError(f"PCD 二进制数据长度异常：{stream.name}（应为 {expected} 字节，实际 {remaining}）")
    return fields, sizes, types, counts, points, encoding


def copy_payload(source, destination, header):
    if header[5] == "binary":
        shutil.copyfileobj(source, destination, length=4 * 1024 * 1024)
        return
    rows = 0
    for line in source:
        stripped = line.strip()
        if not stripped or stripped.startswith(b"#"):
            continue
        if len(stripped.split()) != sum(header[3]):
            raise ValueError(f"PCD ASCII 数据列数不正确：{source.name}，第 {rows + 1} 点")
        destination.write(stripped + b"\n")
        rows += 1
    if rows != header[4]:
        raise ValueError(f"PCD ASCII 点数不正确：{source.name}（应为 {header[4]}，实际 {rows}）")


def merge(prior, relocate, output, force=False):
    prior, relocate, output = (Path(p).expanduser().resolve() for p in (prior, relocate, output))
    if not prior.is_file() or not relocate.is_file():
        raise ValueError("先验与 relocate 输入都必须是已保存完成的 PCD 文件。")
    if os.path.samefile(prior, relocate):
        raise ValueError("两个输入不能是同一个文件。")
    if output in (prior, relocate) or (
        output.exists() and any(os.path.samefile(output, p) for p in (prior, relocate))
    ):
        raise ValueError("输出不能覆盖任一输入地图。")
    if output.exists() and not force:
        raise FileExistsError(f"输出已存在：{output}；将 OVERWRITE_OUTPUT 改为 True（也可使用 --force），或用 -o 指定其他输出路径。")

    with ExitStack() as stack:
        sources = [stack.enter_context(p.open("rb")) for p in (prior, relocate)]
        headers = [read_header(source) for source in sources]
        a, b = headers
        if a[:4] != b[:4] or a[5] != b[5]:
            raise ValueError("两份 PCD 的 FIELDS/SIZE/TYPE/COUNT 和 DATA 编码必须一致。")
        total = a[4] + b[4]
        fields, sizes, types, counts = a[:4]
        output_header = (
            "# .PCD v0.7 - prior + relocate; coordinates copied unchanged\n"
            "VERSION 0.7\n"
            f"FIELDS {' '.join(fields)}\n"
            f"SIZE {' '.join(map(str, sizes))}\n"
            f"TYPE {' '.join(types)}\n"
            f"COUNT {' '.join(map(str, counts))}\n"
            f"WIDTH {total}\nHEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {total}\nDATA {a[5]}\n"
        ).encode("ascii")
        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="wb", prefix=".merge-", suffix=".tmp", dir=output.parent, delete=False
            ) as destination:
                temporary = Path(destination.name)
                destination.write(output_header)
                for path, source, header in zip((prior, relocate), sources, headers):
                    print(f"读取：{path}（{header[4]:,} 点）", flush=True)
                    copy_payload(source, destination, header)
                destination.flush()
                os.fsync(destination.fileno())
            if force:
                os.replace(temporary, output)
            else:
                # Atomically install the finished file, without overwriting a
                # destination another process may have created during the copy.
                os.link(temporary, output)
            print(f"已保存：{output}\n总点数：{total:,}（保留重叠点）", flush=True)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
    return total


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prior", nargs="?", type=Path, default=PCD_DIR / "priori" / PRIOR_PCD_NAME,
                        help="可选覆盖：先验 PCD 路径，默认按顶部文件名读取 PCD/priori/")
    parser.add_argument("relocate", nargs="?", type=Path, default=PCD_DIR / "relocate" / RELOCATE_PCD_NAME,
                        help="可选覆盖：relocate PCD 路径，默认按顶部文件名读取 PCD/relocate/")
    parser.add_argument("-o", "--output", type=Path,
                        help="可选覆盖：输出路径，默认保存到 PCD/merge/merged_<先验时间>_<relocate时间>.pcd")
    parser.add_argument("--force", action="store_true", default=OVERWRITE_OUTPUT,
                        help="允许替换已有输出，始终禁止覆盖输入；默认取顶部 OVERWRITE_OUTPUT")
    args = parser.parse_args()
    prior_time = args.prior.stem.removeprefix("map_")
    relocate_time = args.relocate.stem.removeprefix("map_")
    output = args.output or PCD_DIR / "merge" / f"merged_{prior_time}_{relocate_time}.pcd"
    try:
        merge(args.prior, args.relocate, output, args.force)
    except (OSError, ValueError) as error:
        print(f"拼接失败：{error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("已取消拼接，输入地图未修改。", file=sys.stderr)
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
