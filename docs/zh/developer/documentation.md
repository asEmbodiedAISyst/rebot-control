# 文档

## 目录结构

```
docs/
  conf.py, index.md      英文站点（默认语言）
  user/                  给使用机械臂的人
  developer/             给修改本库的人
  agents/                给 AI agent
  zh/                    中文站点：有自己的 conf.py，页面与英文相同
  internal/              本地笔记；git 忽略，不构建，不发布
  build.py               构建两个站点、跳转页和 llms.txt 文件
```

- 中文页面和英文页面文件名相同，标题相同，顺序也相同。
- API 参考由 docstring 生成（`sphinx-autodoc2`），输出到 `docs/apidocs/`。它只有英文版，中文站点链接过去。
- `docs/internal/` 放尚未发布的计划、真机测试的原始日志和本地环境的笔记。公开页面不链接到它。

## 构建

```bash
pip install -e ".[docs]"
python docs/build.py            # output in docs/_build/html
python -m http.server 8000 -d docs/_build/html
```

`build.py` 遇到 Sphinx 警告就停。`docs` workflow 运行的是同一个脚本，并把结果部署到 GitHub Pages。

## 写作规则

- 每一页只写给一类读者。`user/` 下的页面不解释内部实现，`developer/` 下的页面不重复使用指南。
- 在 `user/` 里，某个功能有缺陷或有风险，就写在这个功能自己的页面上，用 `warning` 块标出。
- 在 `agents/` 里，规则用祈使句，事实用陈述句。限制写成“条件 + 动作”。不加免责声明。
- 英文：文字允许的地方遵循 ASD-STE100（Simplified Technical English）。一句话只讲一件事，操作步骤不超过 20 词，
  描述不超过 25 词，用主动语态，一个词只表示一个意思。
- 中文：直接用中文写，不要逐句翻译。事实、数字、代码和表格与英文页面保持一致。术语用[术语表](glossary.md)里的。
- 两种语言在同一个提交里一起改。
- 只在有助于读者把本库用对的地方解释背景。
