# Developer Note

This documentation is edited using [MkDocs](https://www.mkdocs.org/). Please install it before editing.

## Adding Pages

Because the documentation will include some navigation headers, you may need to edit the configuration file first.

The `mkdocs.yaml` configuration file looks like:

```yaml
site_name: DeepClaw
nav:
    - Home:
      - Overview: index.md
      - Installation: install.md
    - Manual: 
      - DeepClaw Pipeline: pipeline.md
      - ...
    - API:
      - deepclaw: code-api.md
    - Notes:
      - For Developers: devnote.md
theme: readthedocs
```

You can add a new markdown file into docs, like `docs/<NEW_FILE_NAME>.md`. Then edit this configuration file, insert the markdown file into the `nav` , for example:

```yaml
...
nav:
    ...
    - Manual: 
      - <NAVIGATION TAG>: <NEW_FILE_NAME>.md
      - ...
    - ...
```

## Build & Display

CD into `DEEPCLAW_ROOT/doc/deepclawdoc/`

```
$ cd doc/deepclawdoc/
```

Then start MKDOCS serve

```
$ mkdocs serve
INFO    -  Building documentation...
INFO    -  Cleaning site directory
[I 160402 15:50:43 server:271] Serving on http://127.0.0.1:8000
[I 160402 15:50:43 handlers:58] Start watching changes
[I 160402 15:50:43 handlers:60] Start detecting changes
```

Open up `http:/127.0.0.1:8000/` in your browser, and you'll see the default home page being displayed.