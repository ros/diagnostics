^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosdiagnostic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.0 (2021-12-31)
-------------------

1.10.4 (2021-03-14)
-------------------

1.10.3 (2020-12-05)
-------------------

1.10.2 (2020-09-03)
-------------------

1.10.1 (2020-08-20)
-------------------

1.10.0 (2020-08-11)
-------------------

1.9.4 (2020-04-01)
------------------
* noetic release (`#136 <https://github.com/ros/diagnostics/issues/136>`_)
  * Bump CMake version to avoid CMP0048 warning
  Signed-off-by: ahcorde <ahcorde@gmail.com>
  * Changes to make it work with Python3
  Signed-off-by: ahcorde <ahcorde@gmail.com>
  * Use setuptools instead of distutils
  Signed-off-by: ahcorde <ahcorde@gmail.com>
  * Changes from python2 to python3
  Signed-off-by: ahcorde <ahcorde@gmail.com>
  * update keys - diagnostic_common_diagnostics
  Signed-off-by: ahcorde <ahcorde@gmail.com>
  * Minor fixes
  Signed-off-by: ahcorde <ahcorde@gmail.com>
* Contributors: Alejandro Hernández Cordero

1.9.3 (2018-05-02)
------------------

1.9.2 (2017-07-15)
------------------

1.9.1 (2017-07-15)
------------------

1.9.0 (2017-04-25)
------------------
* Initial release
* Created a command to print rosdiagnostics to the console.
  Works very much like rostopic echo but instead automatically connects to the aggregated diagnostics and output in a friendly format the content of the diagnostic report.
  Issue: https://github.com/ros/diagnostics/issues/57
* Contributors: Guillaume Autran
