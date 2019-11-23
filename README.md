# area-coverage-nav
PoC offline coverage path planning for GPS mobile robot navigation

Goal: compute an adequate boustrophedon-like area coverage path based on GPS points defined by the user

Coverage path planning requirements:

- **must** support the coverage of convex and non-convex polygonal shapes
- **must** adapt the coverage to a predefined coverage orientation
- **must** adapt the coverage in the presence of areas to avoid (holes)
- **should** adapt the coverage to a predefined robot footprint
- **should** select an optimal path whenever possible
- **could** support scheduling of multiple area coverage tasks


## Offline coverage path planning

The desired region is converted into a polygon, which can be manipulated in code to achieve a coverage path according to user specifications.

![offline polygon](img/poly_angle_cov.png?raw=true "")

## Angled coverage

The coverage of a region may be adapted to its features. For the PoC, this is purely done based on geometry; future work may look at using computer vision methods to provide better estimates of the path around these features.

![offline polygon](img/angled_coverage_path.png?raw=true "")

## Obstacle avoidance

The coverage path planning may need to account for obstacles in the region of interest.

![offline polygon](img/pond_crop.png?raw=true "")


