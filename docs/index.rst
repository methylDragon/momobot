MOMObot Documentation
======================

..

  Version 1.1.0

This document contains the relevant specifications and software
packages that are used in the MOdular MObile (MOMObot) Robot. MOMObot
is a service AGV built for extensibility and to roam autonomously using ROS!

Generating the documentation
----------------------------

To generate the documentation locally, git clone the repository & build
the document.

.. code:: bash

  $ git clone https://github.com/sutd-robotics/momobot
  $ cd momobot/docs
  $ make html

The mainpage would be located at ``_build/index.html``.

Index
=====

.. toctree::
 :maxdepth: 2
 :caption: Components

 com/hardware
 com/electronics
 com/software

.. toctree::
  :maxdepth: 1
  :caption: Extra

  bonus/vesc_doc
  bonus/cd_links

.. toctree::
   :maxdepth: 1
   :caption: Credits

   credits/contributions
   credits/license
