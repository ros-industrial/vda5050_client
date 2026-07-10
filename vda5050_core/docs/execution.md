# vda5050_core::execution

## 1. Communication Layer

The `ProtocolAdapter` handles the external communication by abstracting MQTT
topic structure and JSON serialization, allowing the users to work with C++
structs.

```cpp
  // Initialize the MQTT client with broker details and identity
  auto mqtt_client = vda5050_core::mqtt_client::create_default_client(
    "tcp://localhost:1883", "client");

  // Create the prototocl adapter to manage VDA5050 headers and topic naming
  auto protocol_adapter = vda5050_core::execution::ProtocolAdapter::make(
    mqtt_client, "uagv", "v2", "ROS-I", "S001");

  // Subscribe to topic just by specifying types. The protocol adapter handles
  // the JSON parsing internally
  protocol_adapter->subscribe<vda5050_core::types::Order>(
    [&](
      vda5050_core::types::Order order, std::optional<vda5050_core::types::Error> error) {
        if (error.has_value())
        {
          // Error handling if the incoming MQTT payload was malformed
        }
        else
        {
          // Application logic for a valid order
          VDA5050_INFO_STREAM("Received order with ID: " << order.order_id);
        }
      }
    },
    0);

  // Publish a connection request. The protocol adapter automatically fills in
  // the header along with a timestamp
  vda5050_core::types::Connection connection;
  connection.connection_state = vda5050_core::types::ConnectionState::ONLINE;
  c->protocol_adapter_->publish<vda5050_core::types::Connection>(connection, 1);
```

## 2. Execution Layer

The Execution Layer separates state and logic where logic is encapsulated into
Strategies that use their internal Engines to manage commands (Events) and
asynchronous synchronization (Wait).

### 2.1 Defining Custom Data

```cpp
  using namespace vda5050_core::execution;

  // Updates are inbound broadcasts from the downstream entities
  struct OrderUpdate
  : public vda5050_core::execution::Initialize<OrderUpdate, UpdateBase>
  {
    std::string order_id;
    uint32_t sequence_id;

    OrderUpdate(const std::string& order_id, uint32_t sequence_id)
    : order_id(std::move(order_id)), sequence_id(sequence_id)
    {
      // Nothing to do here ...
    }
  };

  // Events are outbound commands to internal entities
  struct OrderEvent
  : public vda5050_core::execution::Initialize<OrderEvent, EventBase>
  {
    std::string order_id;
    uint32_t sequence_id;
    std::string node_id;

    OrderEvent(
      const std::string& order_id, uint32_t sequence_id,
      const std::string& node_id)
    : order_id(std::move(order_id)),
      sequence_id(sequence_id),
      node_id(std::move(node_id))
    {
      // Nothing to do here ...
    }
```

### 2.2 Implementing a Strategy

Strategies are reactive and can pause their own logic to wait for specific
updates without blocking other system components.

```cpp
  using namespace vda5050_core::execution;

  class OrderDispatchStrategy : public vda5050_core::execution::StrategyInterface
  {
  public:
    void init(
      std::shared_ptr<vda5050_core::execution::ContextInterface> context) override
    {
      // Register how to handle internal events
      engine()->on<OrderEvent>([](auto event) {
        // Dispatch command to downstream listener
      });

      // Register how to notify engine about update
      context->provider()->on<OrderUpdate>([this](auto update) {
        this->engine()->notify(update);
      });
    }

    void step(
      std::shared_ptr<vda5050_core::execution::ContextInterface> context) override
    {
      // Check wait status
      if (engine()->waiting()) return;  // Exit if still waiting for downstream update

      // Process engine queue
      engine()->step();

      // Query the context for the latest update
      if (auto order_update = context->get_update<OrderUpdate>())
      {
        if (order_update->sequence_id == 1)
        {
          // Emit event to execution queue
          engine()->emit<OrderEvent>(Priority::NORMAL, "order_1", 2, "node_2");

          // Flush execution queue to trigger dispatch
          engine()->step();

          // Pause the strategy until appropriate update arrives
          engine()->suspend_till<OrderUpdate>(
            std::chrono::seconds(5), [](auto update) {
              return update->order_id == "order_1" && update->sequence_id == 2;
            });
        }
      }
    }
  };
```

### 2.3 Implementing a Context

The Strategy uses Context to query system states before performing logic.
Instead of reacting to a stream of data, it retreives data in a type-safe
manner.

```cpp
  using namespace vda5050_core::execution;

  class SimpleContext : public vda5050_core::execution::ContextInterface
  {
  public:
    void init() override
    {
      // Listen to updates and cache them automatically
      provider()->on<OrderUpdate>([this](auto update) {
        std::lock_guard<std::mutex> lock(update_mutex_);
        updates_[update->get_type()] = update;
      });

      // Resources should be created and stored at the start of context
      // initialization and should be assumed as static data
      auto config = std::make_shared<ConfigResource>("agv_1");
      std::lock_guard<std::mutex> lock(resource_mutex_);
      resources_[config->get_type()] = config;
    }

  protected:
    std::shared_ptr<UpdateBase> get_update_raw(
      std::type_index type) const override
    {
      std::lock_guard<std::mutex> lock(update_mutex_);
      auto it = updates_.find(type);
      if (it != updates_.end()) return it->second;
      return nullptr;
    }

    std::shared_ptr<ResourceBase> get_resource_raw(
      std::type_index type) const override
    {
      std::lock_guard<std::mutex> lock(resource_mutex_);
      auto it = resources_.find(type);
      if (it != resources_.end()) return it->second;
      return nullptr;
    }

  private:
    // Maps to store type-erased updates and resources
    std::unordered_map<std::type_index, std::shared_ptr<UpdateBase>> updates_;
    mutable std::mutex update_mutex_;

    std::unordered_map<std::type_index, std::shared_ptr<ResourceBase>> resources_;
    mutable std::mutex resource_mutex_;
  };
```

### 2.4 Orchestration using the Handler

```cpp
  int main()
  {
    auto context = std::make_shared<SimpleContext>();
    auto strategy = std::make_shared<OrderDispatchStrategy>();
    auto handler = Handler::make(context, strategy);

    // Simulate an incoming order update
    context->provider()->push<OrderUpdate>("order_1", 1);

    // Run execution loop
    // First spin will trigger strategy to send an order event
    handler->spin_once();

    // Simulate hardware finishing a task
    context->provider()->push<OrderUpdate>("order_1", 2);

    // Next spin will resolve the wait and continue the strategy
    handler->spin_once();

    return 0;
  }
```
