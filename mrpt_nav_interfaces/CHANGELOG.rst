^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_nav_interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.1 (2024-10-12)
------------------

2.2.0 (2024-09-25)
------------------
* Merge pull request `#149 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/149>`_ from mrpt-ros-pkg/feature/utm-coordinates
  Support UTM global coordinates for geo-referenciated maps
* Add new msg GeoreferencingMetadata.msg
* Update URL entries in package.xml to each package proper documentation
* Contributors: Jose Luis Blanco-Claraco

2.1.1 (2024-09-02)
------------------

2.1.0 (2024-08-08)
------------------
* Add two new service definitions: MakePlanTo, MakePlanFromTo
* Merge branch 'ros2' into wip/port-tps-astar
* Contributors: Jose Luis Blanco-Claraco

2.0.1 (2024-05-28)
------------------
* fix missing <depend> on action_msgs (should fix build on Humble)
* Contributors: Jose Luis Blanco-Claraco

2.0.0 (2024-05-28)
------------------
* Define map_server services
* Add new package mrpt_nav_interfaces with action and msg defintions for navigation
* Contributors: Jose Luis Blanco-Claraco
