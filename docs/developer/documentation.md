# Documentation

## Layout

```
docs/
  conf.py, index.md      English site (the default language)
  user/                  for people who run an arm
  developer/             for people who change the library
  agents/                for AI agents
  zh/                    Chinese site: its own conf.py and the same pages
  internal/              local notes; ignored by git, never built, never published
  build.py               builds both sites, the redirect page and the llms.txt files
```

- The English pages and the Chinese pages have the same file names and the same headings in the same order.
- The API reference is generated from the docstrings (`sphinx-autodoc2`) into `docs/apidocs/`. It exists in English
  only, and the Chinese site links to it.
- `docs/internal/` holds unreleased plans, raw real-arm session logs and notes about the local setup. Public pages
  do not link to it.

## Build

```bash
pip install -e ".[docs]"
python docs/build.py            # output in docs/_build/html
python -m http.server 8000 -d docs/_build/html
```

`build.py` stops on a Sphinx warning. The `docs` workflow runs the same script and deploys the result to GitHub
Pages.

## Writing rules

- Write for one reader. A page in `user/` does not explain internals, and a page in `developer/` does not repeat the
  user guide.
- In `user/`, say on the page of a feature when the feature has a defect or a risk. Use a `warning` block.
- In `agents/`, write rules as imperatives and facts as statements. Give a limit as a condition and an action. Do
  not add disclaimers.
- English: follow ASD-STE100 (Simplified Technical English) where the text allows it. One idea per sentence, at
  most 20 words in an instruction and 25 in a description, active voice, one word for one meaning.
- Chinese: write the page in Chinese, do not translate sentence by sentence. Keep the facts, the numbers, the code
  and the tables the same as the English page. Use the terms in the [glossary](glossary.md).
- Change both languages in the same commit.
- Explain background only where it helps the reader to use the library correctly.
