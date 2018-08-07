# Introduction to dt-shell {#dt-shell-intro status=draft}

Maintainer: Andrea Daniele

Duckietown Shell (dt-shell) is a pure Python utility for Duckietown.

The idea is that most of the functionality is implemented as Docker containers, and
dt-shell provides a nice interface for that, so that user should not type a very long
docker run command line.



## Dependencies

Installing dt-shell via `pip` will also install the following packages

- `GitPython` - A python library used to interact with Git repositories
- `texttable` - A module to generate a formatted text table, using ASCII characters.



## Installation

You can install dt-shell by running

    $ pip install duckietown-shell



## Run the shell

You can launch the shell by running the command

    $ dt

You should see the dt-shell prompt

```
dt>

```
