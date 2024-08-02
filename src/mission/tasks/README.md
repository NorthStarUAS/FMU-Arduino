# Python based mission/task system porting

Here is the current list of python-language mission/task elements.  The thought
here is to begin with all the existing python code and remove the python
versions as the functionality is migrated to the C++ system.

Some of the python tasks (mostly the global tasks) may shift over to be part of
the standard code -- like throttle_safety, switches, fcsmode -- since these are
things that always run anyway.