MicroPython Documentation
=========================

The MicroPython documentation can be found at:
http://docs.micropython.org/en/latest/

The documentation you see there is generated from the files in the docs tree:
https://github.com/micropython/micropython/tree/master/docs

Building the documentation locally
----------------------------------

If you're making changes to the documentation, you may want to build the
documentation locally so that you can preview your changes.

Install Sphinx, and optionally (for the RTD-styling), sphinx_rtd_theme,
preferably in a virtualenv:

     pip install sphinx
     pip install sphinx_rtd_theme

In `micropython/docs`, build the docs:

    make MICROPY_PORT=<port_name> html

Where `<port_name>` can be `unix`, `pyboard`, `wipy`, `esp8266`, or `openmvcam`.

You'll find the index page at `micropython/docs/build/<port_name>/html/index.html`.
