
# Offset calculator
Paczka `offset_calculator` zawiera node o takiej samej nazwie liczący liniowe i kątowe odchylenie od wyznaczonej ścieżki. Odchylenia na prawo są dodatnie, a na lewo ujemne. Wyniki są publikowane w topicach `linear_offset` i `angular_offset`.

## `offset_calculator`

## Subscribed topics
`closest_path_points`([nav_msgs/Path](docs.ros.org/melodic/api/nav_msgs/html/msg/Path.html))

`tf`([tf/tfMessage](http://docs.ros.org/melodic/api/tf/html/msg/tfMessage.html))

Required transforms:

`map` -> `base_link`
## Published topics
`position_offset`([std_msgs/Float64](http://docs.ros.org/api/std_msgs/html/msg/Float64.html))

`heading_offset`([std_msgs/Float64](http://docs.ros.org/api/std_msgs/html/msg/Float64.html))

## Parameters
`~path_approximation_by_parabola` (`bool`, default: 0) Domyślnie ścieżka jest przybliżana prostą łączącą dwa pierwsze punkty ścieżki. Jeśli 1, to ścieżka jest przybliżana wielomianem drugiego stopnia.
