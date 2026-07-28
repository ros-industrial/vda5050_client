# Design Document for `vda5050_core::execution`

This document outlines the architectural design and key components of the
`vda5050_core::execution` library. The goal is to provide a decoupled, type-safe,
and thread-safe library for processing VDA5050 messages and executing robot
commands.

## Overall Architecture

The system follows a Reactive Strategy Pattern by separating the Strategy
(logic to be executed) from the Handler (orchestration for the logic) and
the Context (data storage of the system).

Design Principles:

1. Stateful Context: Instead of transient stream of messages, the system
maintains a snapshot.

1. Reactive Execution: The system sleeps when idle and wakes up only when
new data arrives or timeout occurs.

1. Non-Blocking Logic: Strategies use an internal Engine to manage long running
tasks using asynchronous events dispatch and wait-conditions.

## Core Components

### 1. Data Layer (`UpdateBase`, `ResourceBase`, `Provider`, `Context`)

This layer manages the upstream and inbound flow of information.

- `UpdateBase`: It is a lightweight carrier of external data representing
external information entering the system (e.g., an order update from the AGV)
- `ResourceBase`: It is a carrier of persistent data. Unlike Updates,
Resources are typically cached in the Context (e.g., storage for the MQTT
client)
- `Provider`: It can be used to register callbacks for asynchronous broadcast
of data to registered entities.
- `Context`: It is the systems persistent storage that canbe queried by the
orchestration entities to understand the state of the system before making
decisions.



### 2. Logic Layer (`EventBase`, `Strategy`, `Engine`)

This layer manages the downstream and internal flow of commands.

- `EventBase`: It is a lightweight carrier of commands that are emitted by a
Strategy when it decides the registered entities must do something (e.g.,
motor actuation, navigation, etc.)
- `Strategy`: It evaluates the Updates and Resources found in the Context
and decides which internal Events to emit to the Engine.
- `Engine`: The Strategy's local executor. It manages an Event Queue that
prioritizes and then dispatches internal commands. It also handles Wait
Mechanisms, allowing Strategy to pause until an Update satisfies a
specific condition.



### 3. Execution Layer (`Handler`)

The execution layer provides the orchestration for execution of commands.

- `Handler`: It combines the Strategy and Context together into a single
execution unit.
- A reactive loop manages a condition variable to ensure the Strategy only
steps when necessary, either because a new Update arrived in the Context or a
timeout occured. This prevents busy-waiting and optimizes process usage.



### 4. Communication Layer (`ProtocolAdapter`)

This acts like a translation layer between the raw MQTT network and internal
execution logic.

- It manages VDA5050 headers by automatically handling increment of `headerId`
and timestamps.
- It uses C++ templates and static traits to ensure only valid VDA5050
compliant types are processed.
- It does not manage the MQTT connection itself, instead it accepts a pointer
to `MqttClientInterface`, allowing it to share a connection with other services.

