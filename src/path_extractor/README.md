# Path extractor

Paczka na podstawie aktualnej pozycji samochodu znajduje najblizszy mu punkt na wczesniej wyznaczonej trasie. Program `path_extractor` publikuje wycinek trasy, ktory zawiera ten i sasiednie punkty z trasy, w liczbie zaleznej od parametrow.
Pozostale skrypty i launchfile umozliwiaja przetestowanie glownego programu.

## `path_extractor`

## Published topics
`closest_path_points` ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))

## Required transformations
`map` - `base_link`

## Parameters

- `~path_points_backwards`(`int`, default: 2)
Liczba publikowanych punktow z trasy bezposrednio przed znalezionym najblizszym punktem
- `~path_points_forwards`(`int`, default: 5)
Liczba publikowanych punktow z trasy bezposrednio za znalezionym najblizszym punktem
- `~path_direction`(`bool`, default: 0)
Zmiana kierunku jazdy

## Input file
Informacja o trasie zawarta jest w pliku w formacie pickle, opisanym w dokumentacji paczki `path_generator`.
