std_msgs/Header header
uint64 sequence_id
builtin_interfaces/Time input_stamp
string producer_id
bool success

float32 preprocessing_ms
float32 policy_ms
float32 postprocessing_ms
float32 total_ms

string device
uint64 model_parameters
uint64 cuda_memory_allocated_bytes
uint64 cuda_peak_memory_bytes

string action_encoding
float32[] action
