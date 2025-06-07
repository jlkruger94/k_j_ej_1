# ROS 2 Counter Demo Package

Este repositorio contiene el código funcional del ejercio 1 de programación solicitado por la catedra:

Proveer un paquete de ROS con un launchfile con dos nodos:

Un nodo publica cinco veces por segundo un contador. Este nodo tiene un servicio que resetea su contador.
Un segundo nodo está suscrito al primero y cuando el mensaje con el contador llega a 50 (cada 10 segundos), resetea el contador del nodo.

Ambos nodos pueden configurar por parámetros:

Nodo publicando:
- Frecuencia que publica.
- Cantidad máxima que publica.
Nodo suscrito:
- A qué número reinicia el contador del nodo.

Launchfile que lance todo y permita configurar la frecuencia y el número en el que se reinicia como argumentos.

## Authors

- José Luis Krüger
- Juan Manuel Guariste

## Prerequisites

- ROS 2 Jazzy
- Python 3.8 or higher

## Installation

1. Clone this repository into your ROS 2 workspace:
```bash
cd ~/ros2_ws
mkdir -p src
git clone <repository-url> src/j_k_ej_1
```

2. Build the package:
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Nodes

### Counter Publisher
The counter_publisher node manages and publishes an incrementing counter value.

#### Topics

- `counter_topic` (Publisher)
  - Message type: `std_msgs/Int32`
  - Publishes the current counter value

#### Parameters

- `counter_max` (integer, default: 10)
  - Maximum value the counter will reach before shutting down
  - Can be set via launch file or command line
  - Example: `ros2 run clase5 counter_publisher counter_max:=5`

- `timer_period` (float, default: 1.0)
  - Time interval between counter value publications (in seconds)
  - Can be set via launch file or command line
  - Example: `ros2 run clase5 counter_publisher timer_period:=0.5`

- `reset_arg` (float, default: 50)
  - Maximum count value at which the counter will be reset
  - Can be set via launch file or command line
  - Example: `ros2 run clase5 counter_suscriber reset_arg:=5`

#### Services

- `reset_counter` (std_srvs/Trigger)
  - Resets the counter back to 0
  - Returns success and message when called

To run:
```bash
# Default configuration (counts to 10, publishes every 1 second)
ros2 run clase5 counter_publisher

# Custom configuration (counts to 5, publishes every 0.5 seconds)
ros2 run clase5 counter_publisher --ros-args -p counter_max:=5 -p timer_period:=0.5

# Reset the counter
ros2 service call /reset_counter std_srvs/srv/Trigger "{}"
```

### Counter Subscriber
The counter_subscriber node receives and processes counter values from the publisher.

#### Topics

- `counter_topic` (Subscriber)
  - Message type: `std_msgs/Int32`
  - Receives counter values from the publisher

To run:
```bash
ros2 run clase5 counter_subscriber
```

## Usage

1. Open a terminal and run the publisher:
```bash
ros2 run clase5 counter_publisher
```

2. In a new terminal, run the subscriber:
```bash
ros2 run clase5 counter_subscriber
```

The subscriber will print the counter values as they are received from the publisher.

## Launch File

A launch file is provided that starts both the publisher and subscriber nodes simultaneously:

```bash
ros2 launch clase5 clase5.launch.py
```

This launch file:
- Starts both nodes in the same terminal with a counter value of 5 and timer period of 1.0seconds. 
- Configures both nodes to output to the screen

## Topic Information

- Topic name: `counter_topic`
- Message type: `std_msgs/Int32`
- Publisher: `counter_publisher` node
- Subscriber: `counter_subscriber` node

## Testing

The package includes basic unit tests that can be run using:
```bash
colcon test --packages-select clase5 --event-handlers console_direct+ --pytest-args
```

## License

This package is licensed under the Apache License 2.0.

## Contributing

Please feel free to submit pull requests or open issues for any improvements or fixes.
