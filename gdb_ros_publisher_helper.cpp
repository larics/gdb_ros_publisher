#include <ros/publication.h>
#include <ros/publisher.h>
#include <ros/forwards.h>
#include <ros/subscriber_link.h>
#include <ros/topic_manager.h>
#include <iostream>
#include <dlfcn.h>

namespace ros {

 static void (Publication::*real_publish)(SerializedMessage& m) = nullptr;

 void Publication::publish(SerializedMessage& m)
 {
   // This is a hack to call the real Publication::publish. An alternative would be to just
   // paste the code from publication.cpp.
   if (real_publish == nullptr) {
     void* ptr = dlsym(RTLD_NEXT, "_ZN3ros11Publication7publishERNS_17SerializedMessageE");
     memcpy(&real_publish, &ptr, sizeof(&ptr));
   }
  (*this.*real_publish)(m);
 }

}
