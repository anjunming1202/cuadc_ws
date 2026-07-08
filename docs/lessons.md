# Lessons

> 踩坑记录：环境/工具的坑 + 实验评测的发现，混在一起按时间记，不细分类型
> （分开对两人团队来说分类负担大于收益，见 2026-07-07 的讨论）。只增不改，
> 除非发现记错了；如果以后这份文档写得又长又杂、不同类型的内容互相干扰不
> 好找，再考虑拆分。

## 2026-07-06

- **PDF 中文提取乱码**：`2026中国大学生飞行器设计创新大赛竞赛规则 ★公开★.pdf`
  用 `pdftotext` 提取中文会乱码（字体编码问题）。改用 PyMuPDF
  （`import fitz; fitz.open(path)[i].get_text()`）能正确提取 Unicode。

## 2026-07-07

- **从 Windows 侧 Claude Code 会话桥接到 WSL 时的路径坑**：`wsl.exe -d
  <distro> -- bash -lc '...'` 里如果传了 `/mnt/c/...` 这种 POSIX 风格路径，
  Git Bash 的 MSYS 层会在传给 `wsl.exe` 之前把它悄悄重写成 Windows 路径
  （比如变成 `D:/Program Files/Git/mnt/c/...`），导致找不到文件。加
  `MSYS_NO_PATHCONV=1` 前缀能关掉这个转换。另外多语句的 `-lc` 字符串里，
  变量赋值可能在跨桥接时丢失（`A=hello; echo "$A"` 会打印空），更可靠的
  做法是把脚本写成文件再用 `bash /path/to/script.sh` 执行。
