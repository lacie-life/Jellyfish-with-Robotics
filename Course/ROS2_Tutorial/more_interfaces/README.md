More detail writen CMakeLists.txt :
https://index.ros.org/doc/ros2/Tutorials/Single-Package-Define-And-Use-Interface/#singlepkginterface

Note

You can use an existing interface definition in a new interface definition. For example, let’s say there is a message named Contact.msg that belongs to an existing ROS 2 package named rosidl_tutorials_msgs. Assume that its definition is identical to our custom-made AddressBook.msg interface from earlier.

In that case you could have defined AddressBook.msg (an interface in the package with your nodes) as type Contact (an interface in a separate package). You could even define AddressBook.msg as an array of type Contact, like so:

rosidl_tutorials_msgs/Contact[] address_book
To generate this message you would need to declare a dependency on Contact.msg's package, rosidl_tutorials_msgs, in package.xml:

<build_depend>rosidl_tutorials_msgs</build_depend>

<exec_depend>rosidl_tutorials_msgs</exec_depend>
And in CMakeLists.txt:

find_package(rosidl_tutorials_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES rosidl_tutorials_msgs
)
You would also need to include the header of Contact.msg in you publisher node in order to be able to add contacts to your address_book.

#include "rosidl_tutorials_msgs/msg/contact.hpp"
You could change the call back to something like this:

auto publish_msg = [this]() -> void {
   auto msg = std::make_shared<more_interfaces::msg::AddressBook>();
   {
     rosidl_tutorials_msgs::msg::Contact contact;
     contact.first_name = "John";
     contact.last_name = "Doe";
     contact.age = 30;
     contact.gender = contact.MALE;
     contact.address = "unknown";
     msg->address_book.push_back(contact);
   }
   {
     rosidl_tutorials_msgs::msg::Contact contact;
     contact.first_name = "Jane";
     contact.last_name = "Doe";
     contact.age = 20;
     contact.gender = contact.FEMALE;
     contact.address = "unknown";
     msg->address_book.push_back(contact);
   }

   std::cout << "Publishing address book:" << std::endl;
   for (auto contact : msg->address_book) {
     std::cout << "First:" << contact.first_name << "  Last:" << contact.last_name <<
       std::endl;
   }

   address_book_publisher_->publish(*msg);
 };
Building and running these changes would show the msg defined as expected, as well as the array of msgs defined above.