#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <map>

// Map for movement keys (forward, backward, left, right)
std::map<char, std::vector<float>> moveBindings
{
    {'w', {1, 0}},   // Forward (linear x = 1, angular z = 0)
    {'s', {-1, 0}},  // Backward (linear x = -1, angular z = 0)
    {'a', {0, 1}},   // Left turn (linear x = 0, angular z = 1)
    {'d', {0, -1}},  // Right turn (linear x = 0, angular z = -1)
    {' ', {0, 0}}    // Stop (linear x = 0, angular z = 0)
};

// Reminder message
const char* msg = R"(
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   w
a  s  d

space : stop

CTRL-C to quit
)";

// Init variables
float speed(0.4);   // Linear velocity (m/s) - Reduced default speed
float turn(1.0);    // Angular velocity (rad/s) - Reduced default turn rate
float x(0), th(0); // Forward/backward and angular direction vars
char key(' ');

// For non-blocking keyboard inputs
int getch(void)
{
    int ch;
    struct termios oldt;
    struct termios newt;

    // Store old settings, and copy to new settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Make required changes and apply the settings
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    // Get the current character
    ch = getchar();

    // Reapply old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}

int main(int argc, char** argv)
{
    // Init ROS node
    ros::init(argc, argv, "simple_teleop_twist_keyboard");
    ros::NodeHandle nh;

    // Init cmd_vel publisher
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/robot1/cmd_vel", 10);

    // Create Twist message
    geometry_msgs::Twist twist;

    printf("%s", msg);
    printf("\rUse 'w', 's', 'a', 'd' to move. Space to stop. CTRL-C to quit.\r");

    while (ros::ok())
    {
        // Get the pressed key
        key = getch();

        // If the key corresponds to a key in moveBindings
        if (moveBindings.count(key) == 1)
        {
            // Grab the direction data
            x = moveBindings[key][0];
            th = moveBindings[key][1];

            printf("\rLast command: %c ", key);
        }
        // If ctrl-C (^C) was pressed, terminate the program
        else if (key == '\x03')
        {
            printf("\nQuitting teleop.\n");
            break;
        }
        else
        {
            // Stop the robot for any other key
            x = 0;
            th = 0;
            printf("\rInvalid command! Use 'w', 's', 'a', 'd' or space. ");
        }

        // Update the Twist message
        twist.linear.x = x * speed;
        twist.linear.y = 0;
        twist.linear.z = 0;

        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = th * turn;

        // Publish it and resolve any remaining callbacks
        pub.publish(twist);
        ros::spinOnce();
    }

    return 0;
}
