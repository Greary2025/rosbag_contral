#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import glob
import os


def rename_pcd_files(input_dir, src_digits, dst_digits, dry_run=False):
    """批量重命名PCD文件，支持任意位数转换。"""
    if not os.path.exists(input_dir):
        print(f"错误：目录不存在: {input_dir}")
        return False

    try:
        pcd_files = glob.glob(os.path.join(input_dir, "*.pcd"))

        if not pcd_files:
            print(f"错误：在目录 {input_dir} 中未找到PCD文件")
            return False

        print(f"找到 {len(pcd_files)} 个PCD文件")
        print("开始重命名...")

        renamed_count = 0
        skipped_count = 0

        for pcd_file in sorted(pcd_files):
            dir_path = os.path.dirname(pcd_file)
            filename = os.path.basename(pcd_file)
            name, ext = os.path.splitext(filename)

            if name.isdigit() and len(name) == src_digits:
                new_name = f"{int(name):0{dst_digits}d}{ext}"
                new_path = os.path.join(dir_path, new_name)

                if os.path.exists(new_path) and os.path.abspath(new_path) != os.path.abspath(pcd_file):
                    print(f"跳过：目标已存在 {new_path}")
                    skipped_count += 1
                    continue

                if not dry_run:
                    os.rename(pcd_file, new_path)
                renamed_count += 1

        print("\n重命名完成！")
        print(f"共处理了 {renamed_count} 个文件，跳过 {skipped_count} 个")
        return True

    except Exception as e:
        print(f"重命名过程中发生错误: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description="PCD文件批量重命名工具")
    parser.add_argument("--input-dir", action="append", dest="input_dirs")
    parser.add_argument("--src-digits", type=int, default=5)
    parser.add_argument("--dst-digits", type=int, default=6)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    input_dirs = args.input_dirs or [
        "/mnt/d/rosbag/dynamic/07/pcds",
    ]

    print("=== PCD文件批量重命名工具 ===")
    print(f"将把{args.src_digits}位数字格式改为{args.dst_digits}位数字格式")
    if args.dry_run:
        print("(dry-run 模式，不会实际改名)")
    print("=" * 50)

    for input_dir in input_dirs:
        print(f"\n处理目录: {input_dir}")
        success = rename_pcd_files(input_dir, args.src_digits, args.dst_digits, args.dry_run)

        if success:
            print(f"目录 {input_dir} 处理完成")
        else:
            print(f"目录 {input_dir} 处理失败")

if __name__ == "__main__":
    main()