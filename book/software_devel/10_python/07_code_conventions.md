# Duckietown code conventions {#code-conventions status=ready}


## Python {#code-conventions-python}

### Tabs {#no-tabs}

Never use tabs in Python files.

The tab characters are evil in Python code. Please be *very* careful in changing them.

Do *not* use a tool to do it (e.g. "Convert tabs to spaces"); it will get it wrong.

Better: checked by `what-the-duck`.

### Spaces

Indentation is 4 spaces.

### Line lengths {max-line-length}

Lines should be below 85 characters.

Better: `what-the-duck` report those above 100 characters.

This is just a symptom of a bigger problem.

The problem here is that you do not do how to program well,
therefore you create programs with longer lines.

Do not go and try to shorten the lines; the line length is just the symptom.
Rather, ask somebody to take a look at the code and tell you how to make it better.


### The encoding line

All files must have an encoding declared, and this encoding must be `utf-8`:

    # -*- coding: utf-8 -*-

### Sha-bang lines

Executable files start with:

    #!/usr/bin/env python

### Comments

Comments refer to the next line.

Comments, bad:

    from std_msgs.msg import String # This is my long comment

Comments, better:

    # This is my long comment
    from std_msgs.msg import String


## Logging

For logging, import this logger:

    from duckietown_utils import logger

## Exceptions {status=draft}

    DTConfigException

    raise_wrapped
    compact = True


## Scripts {status=draft}


    def summary():
        fs = get_all_configuration_files()


    if __name__ == '__main__':
        wrap_script_entry_point(summary)

### Imports

Do not do a star import, like the following:

    from rostest_example.Quacker import *
