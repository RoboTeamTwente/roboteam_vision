#include <ros/ros.h>
#include <signal.h>

/**
 * A program that proves that ctrl-c'ing while getting params is bad for business.
 *
 * Expected behavior:
 * If you press ctrl-c when running roboteam_vision, it should exit elegantly.
 *
 * The problem:
 * If you ctrl-c'd roboteam_vision it would sometimes hang, and sometimes
 * even do a core dump.
 *
 * The cause:
 * roboteam_vision runs at 200 Hz, and reads quite a few ros parameters. To honor
 * these requests, ros travels all the way to master, through the abstraction layers,
 * through the XMLRPC stack, straight toward a linux system call to make a network
 * request. I'm not entirely sure, but I read on many parts of the internet that 
 * if you are in a system call ctrl-c is either ignored or incorrectly handled. This
 * means that, if you are getting a ros param, and the user is getting a param, the
 * whole node will be shutdown by the time the system call returns. I believe this causes
 * the hanging/segfaults/core dumps. Normally this is not a problem, because it's quite
 * unlikely the user will ctrl-c during a system call. However, since roboteam_vision
 * runs at 200 Hz and reads quite a few parameters it's a lot more likely to happen.
 *
 * The proof:
 * Set please_crash to true and the program will behave like a regular ros program.
 * If you then compile and run this
 * program, and press ctrl-c, it will hang 99/100 times. Notice that it doesn't do anything
 * but getting a single param as often as possible.
 *
 * The solution:
 * If you set please_crash to false, the program installs its own signal handler, which
 * ensures that ros::shutdown is only called outside a system call (or, more precisely,
 * outside a subroutine that gets any ros params or uses the XMLRPC stack). If you then
 * compile and run this program, the termination of the node does not hang anymore.
 *
 */

namespace {

bool please_shutdown = false;

void mySigIntHandler(int) {
    please_shutdown = true;
}

} // anonymous namespace

int main(int argc, char **argv)
{
    bool please_crash = false;

    if (please_crash) {
        // Init ros.
        ros::init(argc, argv, "crashtest", ros::init_options::AnonymousName); 
    } else {
        ros::init(argc, argv, "crashtest", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler); 

        signal(SIGINT, mySigIntHandler); 
    }

    ros::NodeHandle n;

    while (ros::ok()) {
        bool should_normalize = false;
        ros::param::get("normalize_field", should_normalize);

        if (!please_crash) {
            if (please_shutdown) {
                ros::shutdown();
            }
        }
    }

    return 0;
}
