//
//  Hello World client in C++
//  Connects REQ socket to tcp://localhost:5555
//  Sends "Hello" to server, expects "World" back
//
#include <zmq.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <sstream>

int main ()
{
    //  Prepare our context and socket
    zmq::context_t context (1);
    zmq::socket_t subscriber (context, ZMQ_SUB);

    std::cout << "Connecting to hello world server…" << std::endl;
    subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    subscriber.connect ("tcp://127.0.0.1:5556");

    //  Do 10 requests, waiting each time for a response
    while(1) {
        zmq::message_t update;
        int x_coordinate;            // snprintf((char *) message.data(), 20, "%d", rot_center);

        
        subscriber.recv(&update);
        std::cout << "Received" << std::endl;

        std::istringstream iss(static_cast<char*>(update.data()));
        iss >> x_coordinate;

        std::cout << "The coordinates is: " << x_coordinate << std::endl;
    }
    return 0;
}





// //
// //  Weather update client in C++
// //  Connects SUB socket to tcp://localhost:5556
// //  Collects weather updates and finds avg temp in zipcode
// //
// //  Olivier Chamoux <olivier.chamoux@fr.thalesgroup.com>
// //
// #include <zmq.hpp>
// #include <iostream>
// #include <sstream>
// #include <stdio.h>

// int main (int argc, char *argv[])
// {
//     zmq::context_t context (1);

//     //  Socket to talk to server
//     std::cout << "Collecting updates from weather server…\n" << std::endl;
//     zmq::socket_t subscriber (context, ZMQ_SUB);
//     subscriber.connect("tcp://127.0.0.1:5556");

//     //  Subscribe to zipcode, default is NYC, 10001
//     const char *filter = (argc > 1)? argv [1]: "10001 ";
//     subscriber.setsockopt(ZMQ_SUBSCRIBE, filter, strlen (filter));

//     //  Process 100 updates
//     int update_nbr;
//     long total_temp = 0;
//     for (update_nbr = 0; update_nbr < 100; update_nbr++) {

//         zmq::message_t update;
//         int zipcode, temperature, relhumidity;

//         subscriber.recv(&update);

//         std::istringstream iss(static_cast<char*>(update.data()));
//         iss >> zipcode >> temperature >> relhumidity ;

//         total_temp += temperature;
//     }
//     std::cout     << "Average temperature for zipcode '"<< filter
//                 <<"' was "<<(int) (total_temp / update_nbr) <<"F"
//                 << std::endl;
//     return 0;
// }
