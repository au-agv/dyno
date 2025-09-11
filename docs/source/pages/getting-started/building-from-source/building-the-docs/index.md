# Building the docs

If you are reading this documentation online, it is likely the documentation for
the latest build of DYNO. If you would like to access any of the previous
documentation, you may do so by building the documentation locally as shown in
this section.

The documentation can be built locally after cloning the repository and
activating the bundled Poetry environment. The following instructions apply to
Ubuntu 22.04 Jammy Jellyfish, but may be easily adapted to any Debian-based
distribution and possibly other GNU/Linux operating systems.

Update the package file and install the dependencies in the Poetry virtual
environment:

```shell
poetry lock
poetry install --with documentation
```

Activate the Poetry virtual environment:

```shell
$(poetry env activate)
```

Finally, change directory to `docs` under the root of the repository and run the
`Makefile` to build the DYNO documentation:

```shell
cd docs
make html
```

You may then open the root of the HTML documentation `build/html/index.html` in
your favourite browser.