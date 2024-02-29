# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import subprocess
import sys
sys.path.insert(0, os.path.abspath("../"))

subprocess.call("doxigen Doxyfile.in", shell= True)
show_authors= True

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Assignment_2_RT1'
copyright = '2024, Simone Lombardi'
author = 'Simone Lombardi'
release = '1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions= [
    "sphinx.ext.autodoc",
    "sphinx.ext.doctest",
    "sphinx.ext.intersphinx",
    "sphinx.ext.todo",
    "sphinx.ext.coverage",
    "sphinx.ext.mathjax",
    "sphinx.ext.ifconfig",
    "sphinx.ext.viewcode",
    "sphinx.ext.githubpages",
    "sphinx.ext.napoleon",
    "sphinx.ext.inheritance_diagram",
    "breathe",
]

templates_path = ['_templates']
exclude_patterns = []
autodoc_mock_imports = ["rospy", "assignment_2_2023.msg", "assignment_2_2023.srv", "geometry_msgs.msg", "nav_msgs.msg", "actionlib", "actionlib"]

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

highlight_language = "c++"
source_suffix = ".rst"
master_doc = "index"
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
