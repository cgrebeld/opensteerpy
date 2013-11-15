opensteerpy
===========

OpenSteer is a popular C++ library implementing steering behaviors for autonomous characters in 
games and animation. OpenSteerPy is a Python wrap of OpenSteer? (using Swig).

Some refactoring of the C++ classes was required to make Swig happy.

OpenSteer was initially developed by Craig Reynolds at the Research and Development group 
of Sony Computer Entertainment America.

Building on OSX with macports
============================
- Install python27, swig, swig-python ports
- In src/python:
  - swig -c++ -python -I../../include -o opensteer_wrap.cpp opensteer.i
  - This will generate the opensteer_wrap.cpp + opensteer_wrap.h + opensteer.py files
- Open the xcode project in macosx and build OpenSteerLib target
    - This will produce the product macosx/build/Development/_opensteer.so
    - copy it into the same folder as opensteer.py, and import it like:
    > import opensteer
