# Road Release Process {#road-release-process status=draft}

<div class='requirements' markdown='1'>

Requires: You have implemented a new feature or improved an existing feature in your branch `devel-![project_name]` 

Requires: A robot that is configured and able to follow lanes in Duckietown according to instructions in [](+fall2017_info#checkoff_navigation)

Result: Your branch gets merged into `master` and doesn't break anything

</div>


This page is about what to do once you have developed and tested your new or improved feature and you want to contribute it back to the `master` branch. 

Note: If your branch is not merged into `master` and passes all the tests it will be lost forever.

## Merge `Master` into your branch

    $ git merge origin master

## Unit Tests

TODO: You should write some?

## Continuous Integration

TODO: How to test that your branch is passing the tests? Look [here](https://circleci.com/gh/duckietown) ?

## Regression Tests

If you have developed a perception module you should define a set of regression tests that ensure that it is working properly.

TODO:

## Simulation Tests

TODO: Once the simulator is up and running it should become part of the testing infrastructure.

## Hardware-in-the-loop (HWIL) 

If you have been doing your development on your laptop or some other computer, the time is now to put the code on the RasPI and test.


## Road Test

Verify that the previous functionality of the robot is preserved. For now repeat the instructions in [](+fall2017_info#checkoff_navigation), and ensure that the basic lane following and indefinite navigation abilities are preserved.


## Make a Pull Request

Once all of the tests are passed and you are sufficiently convinced that your code is not going to break the system, you should make a Pull Request by going [here](https://github.com/duckietown/Software/compare?expand=1). For the base branch put `master` and for the compare branch put your branch.
