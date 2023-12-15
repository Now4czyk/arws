# Implementacja prototypu systemu zbioru owoców za pomocą ramienia robotycznego z wizyjnym sprzężeniem zwrotnym

## Opis

Projekt zawiera kompleksowe oprogramowanie, które jest w stanie wykonać zadanie identyfikacji, a następnie zbioru jabłka za pomocą robota UR3e, kamery OAK-D Pro oraz grippera RG2. Po poprawnym montażu wszystkich elementów oraz uruchomieniu wymaganych węzłów, robot ustawi się w pozycji "wypatrywania". Jeśli w polu widzenia kamery znajdzie się jabłko, robot wykona ruch do pozycji zbioru, a następnie zbierze owoc. Po zakończeniu zbioru, robot powróci do pozycji "wypatrywania" i będzie oczekiwał na kolejne jabłko.

## konfiguracja sprzętu

### Robot UR3e

Robota należy podłączyć do routera. Podłączenie naszych komputerów do tego samego routera pozwoli na komunikację z robotem. W konfiguracji robota (na Teach Pendancie) należy zaznaczyć opcję "Enable Remote Control", ustawić pole "Host IP" na adres naszego komputera, oraz dodać do programu robota blok "External Control".

### Kamera OAK-D Pro

Konieczne jest jej umocowanie za pomocą zaprojektowanego przez nas uchwytu. Następnie należy podłączyć ją za pomocą dwóch kabli - jednego zasilającego i drugiego, służącego do transferu danych. Kabel odpowiadający za transmisję danych podłączamy do komputera, na którym będzie uruchomiony węzeł odpowiadający za obsługę kamery.

### Gripper RG2

Należy zamocować go na końcu ramienia robota oraz podłączyć do portu znajdującego się w pobliżu końcówki. Istotnym jest, aby najpierw przymocować kamerę na uchwycie.

## Instalacja

Podaj instrukcje, jak zainstalować i uruchomić projekt. To może obejmować:

1. Wymagania wstępne: ROS2 humble, python3, depthai, ultralytics,
2. Należy zbudować projekt za pomocą komendy `colcon build` oraz `source install/setup.bash`,
3. Następnie wpisujemy komendę `ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=<ADRES_IP_ROBOTA> initial_joint_controller:=scaled_joint_trajectory_controller launch_rviz:=false`,
4. W drugim oknie terminala uruchamiamy moveit2 komendą: `ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e`,
5. W kolejnym oknie terminala uruchamiamy węzeł odpowiedzialny za obsługę kamery: `ros2 run `,

## Użycie

Podaj przykłady użycia projektu. Jeśli jest to biblioteka, pokaż kilka różnych funkcji i jak ich używać. Jeśli jest to aplikacja webowa, opisz, jak ją skonfigurować i używać.

## Wsparcie

Podaj informacje, jak uzyskać pomoc, jeśli ktoś ma problem z projektem.

## Wkład

Jeśli chcesz, aby inni programiści współpracowali z Tobą nad projektem, podaj instrukcje, jak mogą to zrobić. Może to obejmować:

- Jak zgłaszać błędy
- Jak proponować nowe funkcje
- Jak ustawić środowisko deweloperskie

## Licencja

Podaj informacje o licencji, na której oparty jest Twój projekt.