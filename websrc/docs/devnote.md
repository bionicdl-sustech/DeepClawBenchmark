# Developer Note

This documentation is edited using [MkDocs](https://www.mkdocs.org/). For full documentation visit [mkdocs.org](https://www.mkdocs.org).

## Commands

* `mkdocs serve` - Start the live-reloading docs server.
* `mkdocs build` - Build the documentation site.
* `mkdocs -h` - Print help message and exit.


## Adding Pages

Because the documentation will include some navigation headers, you may need to edit the configuration file first.

The `websrc/mkdocs.yaml` configuration file looks like:

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

You can add a new markdown file into docs, like `websrc/docs/<NEW_FILE_NAME>.md`. Then edit this configuration file, insert the markdown file into the `nav` , for example:

```yaml
...
nav:
    ...
    - Manual: 
      - <NAVIGATION TAG>: <NEW_FILE_NAME>.md
      - ...
    - ...
```

## Build

CD into `websrc`

```
$ cd websrc
```

Then build web pages

```
$ mkdocs build -d ../docs --clean
INFO    -  Cleaning site directory
INFO    -  Building documentation to directory: <ROOT>\DeepClawBenchmark\docs
INFO    -  Documentation built in 0.xxx seconds
```

