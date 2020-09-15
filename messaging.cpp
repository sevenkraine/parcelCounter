#include <chrono>
#include <sstream>
#include <stdexcept>
#include <thread>
#include "messaging.h"


const char *bfv::infmt = "j32i8";
const char *bfv::outfmt = "j20c4";

// Time interval (in seconds) between trigger pulses when PLC is in idle mode.
const long bfv::PLC_IDLE_INTERVAL = 30;


// Define the rules of message comparison.
bool operator==(const bfv::BfvMsg &lhs, const bfv::BfvMsg &rhs)
{
    return lhs.BFV_MessageID == rhs.BFV_MessageID
           && lhs.BFV_StatusID == rhs.BFV_StatusID;
}
bool operator!=(const bfv::BfvMsg &lhs, const bfv::BfvMsg &rhs)
{
    return !operator==(lhs, rhs);
}
bool operator==(const bfv::PlcMsg &lhs, const bfv::PlcMsg &rhs)
{
    return lhs.PLC_MessageID == rhs.PLC_MessageID
           && lhs.PLC_TriggerID == rhs.PLC_TriggerID;
}
bool operator!=(const bfv::PlcMsg &lhs, const bfv::PlcMsg &rhs)
{
    return !operator==(lhs, rhs);
}

// Overload output operator allowing for printing out messages.
std::ostream &operator<<(std::ostream &os, const bfv::BfvMsg &msg)
{
    os << "BFV_MessageID: " << msg.BFV_MessageID << std::endl;
    os << "BFV_Status_Timestamp:" << msg.BFV_Status_Timestamp << std::endl;
    os << "BFV_Report_FlowID: " << msg.BFV_Report_FlowID << std::endl;
    os << "BFV_CalibrationID: " << msg.BFV_CalibrationID << std::endl;
    os << "BFV_StatusID: " << msg.BFV_StatusID << std::endl;
    os << "BFV_Length_in_Flow_Direction: " << msg.BFV_Length_in_Flow_Direction << std::endl;
    os << "BFV_Total_Length: " << msg.BFV_Total_Length << std::endl;
    os << "BFV_Total_Width: " << msg.BFV_Total_Width << std::endl;
    os << "BFV_Package_Count:" << msg.BFV_Package_Count << std::endl;
    os << "BFV_Percent_Area_Right: " << msg.BFV_Percent_Area_Right << std::endl;
    os << "BFV_Percent_Area_Left: " << msg.BFV_Percent_Area_Left << std::endl;
    os << "BFV_Status_Code: " << msg.BFV_Status_Code << std::endl;
    return os;
}

std::ostream &operator<<(std::ostream &os, const bfv::PlcMsg &msg)
{
    os << "PLC_MessageID: " << msg.PLC_MessageID << std::endl;
    os << "PLC_TriggerID: " << msg.PLC_TriggerID << std::endl;
    os << "PLC_Timestamp: " << msg.PLC_Timestamp << std::endl;
    os << "PLC_Belt_Speed: " << msg.PLC_Belt_Speed << std::endl;
    os << "PLC_Status_Code: " << msg.PLC_Status_Code << std::endl;
    os << "PLC_Flow_Request: " << msg.PLC_Flow_Request << std::endl;
    os << "PLC_Vision_Calibration_Request: " << msg.PLC_Vision_Calibration_Request << std::endl;
    os << "PLC_Vision_Status_Request: " << msg.PLC_Vision_Status_Request << std::endl;
    os << "PAD_1: " << msg.PAD_1 << std::endl;
    return os;
}



/** \brief Establish connection with PLC.
 *
 */
void bfv::Link::up()
{
    std::stringstream stream;
    stream << module << " " << daemon << " " << inid << " " << outid;
    std::string dest = stream.str();
    printf("[%s]", dest.c_str());
    plc_log_init(PLC_LOG_TTY);

    plc = plc_open(dest.c_str());

    if (plc == NULL) {
        plc_print_error(plc, "plc_open");
        throw std::runtime_error("Error: connecting to PLC failed.\n");
    }
    
    std::thread worker(&bfv::Link::fork, this);
    printf("made it");
    worker.detach();

    is_up = true;
}


/* \brief Terminate the connection with PLC.
 */
void bfv::Link::down()
{
    if (is_up) {
        int err = plc_close(plc);
        if (err == -1) {
            plc_print_error(plc, "plc_close");
        }
    }
    is_up = false;
}

/** \brief Read a message from PLC.
 *
 *  \param[out] msg message from PLC
 */
void bfv::Link::recv(bfv::PlcMsg &msg)
{
    std::lock_guard<std::mutex> guard(msg_mutex);
    msg = *in;
}


/** \brief Send a message to the PLC.
 *
 *  \param[in] msg message for PLC
 */
void bfv::Link::send(bfv::BfvMsg &msg)
{
    msg.BFV_MessageID++;
    msg.BFV_Status_Timestamp = std::time(nullptr);
    std::lock_guard<std::mutex> guard(msg_mutex);
    *out = msg;
}


/** \brief Get the link status.
 *
 *  \return true if link is up, false otherwise
 */
bool bfv::Link::valid()
{
    return is_up;
}


/** \brief Retrive current message for PLC
 *
 *  \return msg message being sent to PCL
 */
bfv::BfvMsg bfv::Link::retrv()
{
    std::lock_guard<std::mutex> guard(msg_mutex);
    return *out;
}


/** \brief Create a worker monitoring communication between PLC and BFV.
 */
void bfv::Link::fork()
{
    while (true) {
        auto start = std::chrono::steady_clock::now();

        // Read a message from PLC.
        {
            auto msg = in.get();
            std::lock_guard<std::mutex> guard(msg_mutex);
            auto err = plc_read(plc, 0, NULL, msg, sizeof * msg, 1, bfv::infmt);
            // We are polling the PLC for pending requests without halting
            // the application. Hence, ignore timeout errors resulting from
            // lack of new messages from the PCL.
            if (err == -1) {
                if (plc->j_error != PLCE_TIMEOUT) {
                    plc_print_error(plc, "plc_read");
                }
            }
        }

        // Send a message to PLC.
        {
            auto msg = out.get();
            std::lock_guard<std::mutex> guard(msg_mutex);
            auto err = plc_write(plc, 0, NULL, msg, sizeof * msg, 0, bfv::outfmt);
            // In slave mode, the PLC controlls when connections are opened
            // or closed. Hence, ignore write errors when no connection is
            // established. Writes will succeed as soon as the PLC reconnects
            // to the enipd.
            if (err == -1) {
                if (plc->j_error != PLCE_NO_ENDPOINT) {
                    plc_print_error(plc, "plc_write");
                }
            }
        }

        auto end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        auto reminder = timelag - elapsed.count();
        if (reminder > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(reminder));
        }
    }
}
