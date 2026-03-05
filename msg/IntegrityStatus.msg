std_msgs/Header header
string check_name
uint8 STATUS_OK=0
uint8 STATUS_WARN=1
uint8 STATUS_CRITICAL=2
uint8 status
string fault_code
string description
float64 measured_value
float64 threshold_exceeded
string units
# recoverable=true: environmental fault; aggregator may auto-recover when cleared.
# recoverable=false: integrity fault; requires explicit /poise/reset.
bool recoverable
