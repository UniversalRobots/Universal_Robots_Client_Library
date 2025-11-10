^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur_client_library
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.6.0 (2025-11-10)
------------------
* RTDEClient sendStart handle missed confirmation (`#403 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/403>`_)
* Add enum value for 3PE input active to UrRtdeSafetyStatusBits (`#401 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/401>`_)
* Save polyscope folder in local filesystem (`#399 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/399>`_)
* Succeed power on command on PolyScope 5 when robot is running (`#397 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/397>`_)
* Bump actions/upload-artifact from 4 to 5 (`#398 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/398>`_)
* Contributors: Andrew C. Morrow, Felix Exner, dependabot[bot]

2.5.0 (2025-10-24)
------------------
* Polyscopex dashboard client (`#392 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/392>`_)
* Use enable_external_ft_sensor on old software versions (`#395 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/395>`_)
* Rtde external ft (`#388 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/388>`_)
* Fixed auto reconnection to the RTDE server. (`#384 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/384>`_)
* Refactor Rtde client test (`#389 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/389>`_)
* Add release.yml for auto-generated changelogs (`#391 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/391>`_)
* Allow an empty input recipe to the RTDE client (`#318 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/318>`_)
* Contributors: Felix Exner, Mads Holm Peters, URJala

2.4.0 (2025-10-13)
------------------
* UR18 support (`#387 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/387>`_)
* Remove output_bit_register_0...63 and input_bit_register_0..63 from RTDE list (`#385 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/385>`_)
* Set force mode parameters from config (`#383 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/383>`_)
* Direct torque control (`#381 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/381>`_)
* Update RTDE list to include new fields (`#380 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/380>`_)
* Add const qualifier to get functions and changed map value retrieval to at() function (`#379 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/379>`_)
* Contributors: Dominic Reber, Felix Exner, Pablo David Aranda Rodriguez, URJala

2.3.0 (2025-09-15)
------------------
* Install endian header on Windows and Apple only (`#372 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/372>`_)
* Add support for UR8 LONG (`#375 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/375>`_)
* Change ubuntu manpage link from bionic to noble (`#374 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/374>`_)
* Bump actions/setup-python from 5 to 6 (`#373 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/373>`_)
* Add possibility to register multiple callbacks to ReverseInterface and TrajectoryPointInterface (`#359 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/359>`_)
* Contributors: Felix Exner, dependabot[bot]

2.2.0 (2025-07-21)
------------------
* Remove print statement when executing optimovel primitives (`#365 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/365>`_)
* Remove SDK version mapping (`#355 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/355>`_)
* Support optimove motions in InstructionExecutor (`#354 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/354>`_)
* Initialize ReverseInterface with a config struct (`#351 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/351>`_)
* Join thread_move instead of killing it (`#349 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/349>`_)
* Fix external_control urcapx version to 0.1.0 for PolyScope 10.7.0 (`#350 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/350>`_)
* Contributors: Felix Exner

2.1.0 (2025-06-18)
------------------
* Minimal support for building on macOS (`#341 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/341>`_)
* Install endian.h and add that to the target include directories on Windows and MacOS (`#345 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/345>`_)
* Add ScriptReader for script template parsing (`#343 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/343>`_)
* Add more tests for VersionInformation (`#344 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/344>`_)
* Fix driver branch for Jazzy downstream build (`#339 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/339>`_)
* Stop control, when UrDriver object is destroyed (`#338 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/338>`_)
* Add new robot types to URSim startup script (`#331 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/331>`_)
* Fix robot message type POPUP (`#335 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/335>`_)
* Disable checking links for two broken ones (`#333 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/333>`_)
* readme: load ROSin imgs from press_kit repository (`#334 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/334>`_)
* Added configuration data to packages parsed from the primary interface (`#327 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/327>`_)
* Correct message sum in test_tool_contact (`#324 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/324>`_)
* Fix the image sizes in architecture section (`#321 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/321>`_)
* Check links using lychee (`#319 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/319>`_)
* Update ROS distributions for industrial_ci (`#317 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/317>`_)
* Support PolyScopeX simulator for 10.8.0 (`#315 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/315>`_)
* Add an API reference page to the docs (`#314 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/314>`_)
* Update documentation (`#309 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/309>`_)

* Contributors: Andrew C. Morrow, Felix Exner, G.A. vd. Hoorn, Mads Holm Peters

2.0.0 (2025-04-16)
------------------
* Add functionality to send MoveP and MoveC instructions to the robot (`#303 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/303>`_)
* Fix naming issues (`#307 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/307>`_)
* Add more tests for start_ursim.sh (`#305 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/305>`_)
* [start_ursim.sh] Use direct web pages instead of GitHub API to download URCap (`#308 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/308>`_)
* Fix typo in freedrive example document (`#304 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/304>`_)
* Always download and install the latest URCap(X) if not present (`#301 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/301>`_)
* Document robot setup for PolyScope X (`#302 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/302>`_)
* Bump bats-core/bats-action from 3.0.0 to 3.0.1 (`#300 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/300>`_)
* Polyscope x integration tests (`#295 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/295>`_)
* Contributors: Felix Exner, dependabot[bot], xndcn

1.9.0 (2025-03-28)
------------------
* Make start_ursim.sh support polyscopex (`#294 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/294>`_)
* Reduce usage of dashboard client in tests and examples (`#296 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/296>`_)
* Try catch RTDE setup (`#285 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/285>`_)
* add missing headers (`#290 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/290>`_)
* PrimaryClient: Add methods to unlock protective stop and stop the program (`#292 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/292>`_)
* Set increased timeout in dashboard client test (`#293 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/293>`_)
* Do not print a warning when querying the dashboard server for a running program (`#287 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/287>`_)
* Primary client power on (`#289 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/289>`_)
* Contributors: Andrei Kholodnyi, Dominic Reber, Felix Exner

1.8.0 (2025-03-17)
------------------
* Remove unused variables (`#288 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/288>`_)
* Remove direct primary and secondary stream from UrDriver (`#283 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/283>`_)
* Configure gcovr to ignore negative hits as errors (`#284 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/284>`_)
* Add an explicit CMake option to turn on/off integration tests (`#282 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/282>`_)
* instruction_executor: Allow canceling an instruction (`#281 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/281>`_)
* instruction_executor: fix movel test (`#280 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/280>`_)
* Fix buffer order of acceleration and velocity (`#279 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/279>`_)
* Support compilation on Windows (`#229 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/229>`_)
* Contributors: Felix Exner, VDm

1.7.1 (2025-02-25)
------------------
* Fix trajectory result in trajectory forward mode when no trajectory is running (`#276 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/276>`_)
* Remove sending an idle command in quintic spline test (`#275 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/275>`_)
* In servo mode always allow targets close to current pose (`#273 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/273>`_)
* Contributors: Felix Exner

1.7.0 (2025-02-19)
------------------
* Make UrDriver tests run without ctest (`#270 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/270>`_)
* UrDriver: Send program in headless mode after creating trajectory and script_command servers (`#271 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/271>`_)
* Improve limit check (`#256 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/256>`_)
* Use colored log output and timestamps in default log handler (`#267 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/267>`_)
* Parametrize reconnection time for UrDriver (`#266 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/266>`_)
  Co-authored-by: Dominic Reber <71256590+domire8@users.noreply.github.com>
* Fix DashboardClient load program from subdir (`#269 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/269>`_)
* Increase dashboard timeout in ExampleRobotWrapper to 10s
* Disable internal deprecation warning
* Use a config struct for initializing UrDriver (`#264 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/pull/264>`_)
* Use ExampleRobotWrapper for initialization in all examples (`#265 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/265>`_)
* Enable nightly CI jobs (`#263 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/263>`_)
* Expose diagnostic error codes (`#225 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/225>`_)
* RTDEClient: pause and stop in destructor only if running (`#257 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/257>`_)
* Use coverage flags to distinguish between runs (`#261 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/261>`_)
* Fix branch name for integration tests run on push (`#262 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/262>`_)
* Add codecov/test-results-action (`#260 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/260>`_)
* Fix GH edit URL for trajectory_point_interface example (`#259 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/259>`_)
* Update URL check
* Show which example is running in run_examples.sh
* Add documentation for all examples (`#258 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/pull/258>`_)
* Update RT setup documentation to point to urcl docs
* Use EXPECT_NEAR vs EXPECT_EQ
* Fix typo in start_ursim.sh help
* Make CI capable to run with urcap
* Use ExampleRobotWrapper in integration tests (`#252 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/pull/252>`_)
* Add a wrapper to handle all robot setup (`#252 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/pull/252>`_)
* Allow clang-format to indent preprocessor directives (`#246 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/246>`_)
* docs: Clarify that the motion functions use script functions for execution (`#255 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/255>`_)
* Update link to sphinx-doc.org using https (`#247 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/247>`_)
* Use joint speed for extrapolation rather than differences (`#254 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/254>`_)
* Move setup instructions to ur_client_library (`#248 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/248>`_)
* Add more information about acceleration/velocity parametrization in trajectory examples (`#251 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/251>`_)
* Contributors: Felix Exner, Rune Søe-Knudsen, jessica-chen-ocado, Dominic Reber

1.6.0 (2025-01-23)
------------------
* Do not throw exception in DashboardClient::sendRequest (`#249 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/249>`_)
* Add instruction executor for high-level robot control (`#242 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/242>`_)
* Modernize cmake (`#244 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/244>`_)
* Update links to dashboard server documentation (`#243 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/243>`_)
* Trajectory point velocities and example (`#241 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/241>`_)
* Updated documentation (`#228 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/228>`_)
* Update ci (`#239 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/239>`_)
* Enable force mode compatibility with various move types (`#230 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/230>`_)
* Update package maintainers (`#238 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/238>`_)
* Bump codecov/codecov-action from 3 to 5 (`#234 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/234>`_)
* Remove the not regarding MIT license (`#237 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/237>`_)
* Bump pre-commit/action from 3.0.0 to 3.0.1 (`#236 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/236>`_)
* Bump actions/checkout from 1 to 4 (`#232 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/232>`_)
* Bump actions/setup-python from 4 to 5 (`#235 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/235>`_)
* Bump actions/upload-artifact from 3 to 4 (`#233 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/233>`_)
* Add dependabot configuration to update actions (`#231 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/231>`_)
* Contributors: Felix Exner, Rune Søe-Knudsen, dependabot[bot]

1.5.0 (2024-11-25)
------------------
* Adapt RTDE output recipe based on robot response (`#221 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/221>`_)
* CI: Fix flaky example runs (`#223 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/223>`_)
* Giving force mode parameters as arguments when calling startForceMode (`#208 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/208>`_)
* Add more arguments to start_ursim.sh (`#220 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/220>`_)
* Tcp socket improvements (`#222 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/222>`_)
* Added family photo to readme (`#219 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/219>`_)
* Add missing algorithm include (`#214 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/214>`_)
* Added missing RTDE data packages and fixed incorrect names (`#213 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/213>`_)
* Contributors: Felix Exner, Remi Siffert, URJala

1.4.0 (2024-09-10)
------------------
* Ensure that the targets are reachable within the robots limits (`#184 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/184>`_)
* Analog domain (`#211 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/211>`_)
* Fix clang compilation error (`#210 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/210>`_)
* Moved reset of speed slider to correct teardown function, so that it … (`#206 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/206>`_)
  …resets between each test.
* [doc] Fix syntax in example.rst (`#207 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/207>`_)
* [doc] Migrate documentation to sphinx (`#95 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/95>`_)
* Contributors: Felix Exner, Mads Holm Peters, Remi Siffert, URJala

1.3.7 (2024-06-03)
------------------
* [ci] Update CI
  * Run downstream tests for ICI
  * Correctly name jobs
  * Test Jazzy driver
* [start_ursim] Add program directory at correct location also when no model is provided
* [start_ursim] Always check ursim version for compatibility
* [start_ursim] Use a program folder per model
* [ci] Update distros for prerelease test
* Contributors: Felix Exner, Vincenzo Di Pentima

1.3.6 (2024-04-04)
------------------
* Changed spline interpolation to use the last commanded joint velocity… (`#195 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/195>`_)
* Contributors: Mads Holm Peters, Rune Søe-Knudsen

1.3.5 (2024-02-23)
------------------
* Add support for UR30 in start_ursim.sh (`#193 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/193>`_)
* Add header guard to datatypes.h (`#189 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/pull/189>`_)
* Remove duplicated entry in clang-format file (`#188 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/pull/188>`_)
* Wait after docker kill to prevent name conflicts (`#187 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/187>`_)
* Contributors: Felix Exner, Robert Wilbrandt

1.3.4 (2023-09-22)
------------------
* Make depreaction warning for keepalive_counter a warning instead of error (`#182 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/182>`_)
* Added watchdog configuration for the reverse socket (`#178 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/178>`_)
* Add support for ur20 in start_ursim script (`#179 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/179>`_)
* Use pre-commit for clang-format (`#175 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/175>`_)
* Make tcp_server retry binding the socket (`#176 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/176>`_)
* Contributors: Felix Exner, Mads Holm Peters

1.3.3 (2023-09-05)
------------------
* Add support for setting socket max num tries and reconnect timeout (`#172 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/172>`_)
* Unify socket open (`#174 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/174>`_)
* Added handling of spline interpolation with end point velocities (`#169 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/169>`_)
* Throws exception if the URScript file doesn't exists (`#173 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/173>`_)
* Added check to ensure receive timeout isn't overwritten (`#171 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/171>`_)
* Added RTDEClient constructor with vector recipes (`#143 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/143>`_)
* Only warn if system is not setup for FIFO scheduling (`#170 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/170>`_)
* Ensuring that the Timestamp is always in the output recipe (`#168 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/168>`_)
* CI: Add Iron to CI tests (`#167 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/167>`_)
* Add issue templates for bugs and features (`#166 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/166>`_)
* Updated license (`#164 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/164>`_)
* Bugfixes for spline interpolation (`#162 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/162>`_)
   * Add separate rounding in the conversion from float to int32
   * Add more debug printout for splines
   * Add Copying flight reports if CI fails
   * Update ursim mininum version in start_ursim.sh
* Fix the prerelease ci for Melodic (`#163 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/163>`_)
* Contributors: Dag-Are Trydal, Felix Exner, Felix Exner (fexner), Mads Holm Peters, Michael Eriksen, RobertWilbrandt, Rune Søe-Knudsen, urmahp, urrsk

1.3.2 (2023-07-13)
------------------
* Add a cmake option to activate address sanitizers (`#146 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/146>`_)
* Install start ursim (`#155 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/155>`_)
* Add spline interpolation on robot (`#151 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/151>`_)
* Add codecov.yml to exclude test and examples folders (`#152 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/152>`_)
* Make URSim log files available as artifacts also for the CI-industrial (`#153 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/153>`_)
* Remove Foxy from CI
* Add a script to run the examples instead of run-parts
* Add SaveLog command to the Dashboard client
* Make URSim log files available as artifacts
* Specifically set RTDE pipeline producer to FIFO scheduling (`#139 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/139>`_)
* Added support for force_mode, freedrive and tool contact (`#138 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/138>`_)
* Docs: Update link to ros_industrial_cmake_boilerplate
* Added tests for the comm classes (`#129 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/129>`_)
* Changed num_retries from static to an unsigned int (`#136 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/136>`_)
* Build downstream humble version from humble branch (`#132 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/132>`_)
* Contributors: Felix Exner, Mads Holm Peters, Rune Søe-Knudsen, Robert Wilbrandt

1.3.1 (2022-11-30)
------------------
* CI: Add a prerelease check that calls `bloom-generate` (`#134 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/134>`_)
* Contributors: Felix Exner

1.3.0 (2022-11-28)
------------------
* Dashboard commands, Docker Image and CI step for running the examples `#127 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/127>`_
* Added tests for the rtde interface clasess (`#125 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/125>`_)
* Fix unique_ptr type (`#124 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/124>`_)
* Fix 'BEGIN_REPLACE' - used in tool_communication (copy `#101 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/101>`_) (`#120 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/120>`_)
  (cherry picked from commit f7ce9f73181848f3957c660647fac0e5325862b9)
  Co-authored-by: rxjia <60809735+rxjia@users.noreply.github.com>
* Contributors: Felix Exner, Mads Holm Peters, RobertWilbrandt, Rune Søe-Knudsen, mergify[bot], urmarp, urrsk

1.2.0 (2022-10-04)
------------------
* Initialized receive timeout and changed exception to warning (`#118 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/118>`_)
* Added tests for the control interface classes (`#112 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/112>`_)
* Added note about Polyscope version requirement
* Added tcp_offset
* Added interface for forwarding script commands to the robot, that is … (`#111 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/111>`_)
* Fixed parsing of incomming packages when using rtde protocol v1 (`#114 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/114>`_)
  The received rtde packages should be parsed slightly different whether we use protocol v1 or v2.
* Add codecov step (`#116 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/116>`_)
* Added humble build
* Fixed downstream test instructions
* Update atomicops.h (`#117 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/117>`_)
  Fix the url in the comment regarding POSIX semaphores to fix error in the CI
* Make the read during boot depend on the frequency of the robot controller (`#102 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/102>`_)
* Ignore debian folder in check_links (`#100 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/100>`_)
  Otherwise this job raises an error in the release repository.
* Contributors: Felix Exner, Mads Holm Peters, Rune Søe-Knudsen, urmahp, urmarp

1.1.0 (2022-04-22)
------------------
* Support starting the driver, before the robot is booted (`#98 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/98>`_)
* Clear the queue when consumer reads from it (`#96 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/96>`_)
* Fix build with newer glibc
* Doxygen check (`#77 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/77>`_)
* Added target_frequency to RTDEClient (`#85 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/85>`_)
* Removed console_bridge dependency (`#74 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/74>`_)
* Added "On behalf of Universal Robots A/S" notice (`#81 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/81>`_)
  to all files that have been created by FZI
* Always install package.xml file (`#78 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/78>`_)
* register package with ament index
* Corrected smaller doxygen errors
* Added rosdoc_lite check
* Contributors: Cory Crean, Felix Exner, Jørn Bersvendsen, Mads Holm Peters, Martin Jansa, Stefan Scherzinger

1.0.0 (2021-06-18)
------------------
* Added Cartesian streaming interface `#75 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/75>`_
* Added trajectory forwarding interface `#72 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/72>`_
* Refactored Reverse interface `#70 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/70>`_ from fmauch/refactor_reverse_interface
* Added option for robot_ip as runtime argument for rtde_test (`#71 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/71>`_)
* Added reverse_ip parameter (`#52 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/52>`_)
* Move calibration check out of constructor. `#65 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/65>`_ from fmauch/calibration_check_optional
* Install the resources folder instead of the script file directly (`#62 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/62>`_)
* Use a non-blocking tcp server for the `ReverseInterface` and `ScriptSender`. `#46 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/46>`_ from fmauch/tcp_server
* Added LogHandler `#40 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/40>`_ from urmahp/logging_feature
* Fixed links in README (`#35 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/35>`_)
* Contributors: Felix Exner, G.A. vd. Hoorn, JS00000, Lennart Puck, Mads Holm Peters, Tristan Schnell

0.1.1 (2020-09-15)
------------------
* readme: missing whitespace
* Further elaborated license statements in README
* Install package.xml when built with catkin support
* Contributors: Felix Exner, G.A. vd. Hoorn

0.1.0 (2020-09-11)
------------------
* initial standalone release
