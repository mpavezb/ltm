# Proposed work for future releases


## Documentation

- Inline documentation for methods.
- ROS API documentation.
- Pluginlib documentation.


## Usability

- Clean up terminology for methods and API.
- Manage Where.msg information as an entity. The location message should be defined by the user, and kept on its own collection.


## Functionality

- Database migration for updated entities (avoid losing old DB entries).
- Automatical updates for the historical and generical relevances.
- Server with warnings when resource usage reaches the available limits.


## Maintainability

- Migrate the `ltm_db` and `ltm` mongo interface from the legacy C++ driver to the latest version (v3.3 at the moment).


## Scalability and Efficiency

- Disk usage mitigation.
- Streams degradation.
- MongoDB queries analysis.
