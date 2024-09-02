# -*- coding: utf-8 -*-

import os, sys, time
from os.path import abspath, dirname, join

from docutils import nodes
from sphinx.ext import intersphinx

import catkin_pkg.package

catkin_dir = dirname(dirname(abspath(__file__)))
catkin_package = catkin_pkg.package.parse_package(join(catkin_dir, catkin_pkg.package.PACKAGE_MANIFEST_FILENAME))

project = catkin_package.name
copyright = time.strftime("%Y") + ', Czech Technical University in Prague'
author = ", ".join([a.name for a in catkin_package.authors])
version = catkin_package.version
release = catkin_package.version

import catkin_sphinx
html_theme_path = [catkin_sphinx.get_theme_path()]

# Use ROS theme
html_theme = 'ros-theme'

# Add any Sphinx extension module names here, as strings. They can be extensions
# coming with Sphinx (named 'sphinx.ext.*') or your custom ones.
extensions = ['sphinx.ext.autodoc', 'sphinx.ext.intersphinx', 'sphinx.ext.extlinks',
              'sphinx.ext.viewcode']

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# The suffix of source filenames.
source_suffix = '.rst'

# The master toctree document.
master_doc = 'index'

# Resolve ROS message and service dependencies
ros_distro = os.getenv('ROS_DISTRO', 'latest')
ros_api_base = 'https://docs.ros.org/en/%s/api/'
ros_api = ros_api_base % ros_distro

# Example configuration for intersphinx: refer to the Python standard library.
python_version = '{}.{}'.format(sys.version_info.major, sys.version_info.minor)
intersphinx_mapping = {
    'python': ('https://docs.python.org/{}/'.format(python_version), None),
    'genpy': (ros_api + 'genpy/html', None),
}

extlinks = {
    'diagnostic_msgs': (ros_api + 'diagnostic_msgs/html/msg/%s.html', 'diagnostic_msgs/'),
}

# Produce both class and constructor doc comments
autoclass_content = 'both'


def ros_msg_reference(app, env, node, contnode):
    text_node = next(iter(contnode.traverse(lambda n: n.tagname == '#text')))
    parts = text_node.astext().split('.')
    # diagnostic_msgs.msg.DiagnosticStatus
    if len(parts) == 3:
        pkg, obj_type, obj = parts
    # diagnostic_msgs.msg._DiagnosticStatus.DiagnosticStatus
    elif len(parts) == 4:
        pkg, obj_type, module, obj = parts
        if "_" + obj != module:
            return intersphinx.missing_reference(app, env, node, contnode)
    else:
        return intersphinx.missing_reference(app, env, node, contnode)
    if obj_type not in ("msg", "srv"):
        return intersphinx.missing_reference(app, env, node, contnode)
    target = ros_api + '{}/html/{}/{}.html'.format(pkg, obj_type, obj)
    ref_node = nodes.reference()
    ref_node['refuri'] = target
    title = '{}/{}'.format(pkg, obj)
    text_node = nodes.literal(title, title)
    text_node['classes'] = ['xref', 'ros', 'ros-' + obj_type]
    ref_node += text_node
    return ref_node


# Backport of fix for issue https://github.com/sphinx-doc/sphinx/issues/2549. Without it, :ivar: fields wrongly resolve cross-references.
from sphinx.domains.python import PyObject
PyObject.doc_field_types[list(map(lambda f: f.name == 'variable', PyObject.doc_field_types)).index(True)].rolename = None


def setup(app):
    app.connect("missing-reference", ros_msg_reference)

