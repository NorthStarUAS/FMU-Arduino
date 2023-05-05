#include "../../setup_board.h"

#include "serial.h"

class relay_t {

public:

    enum class dest_enum {
        host_dest,
        gcs_dest,
    };

    relay_t();
    ~relay_t();

    void set_gcs_link( SerialLink *link ) {
        gcs_link = link;
    }
    void set_host_link( SerialLink *link ) {
        host_link = link;
    }
    int forward_packet( dest_enum dest, uint8_t id, uint8_t *buf, uint16_t buf_size );

private:

    SerialLink *gcs_link = nullptr;
    SerialLink *host_link = nullptr;

};

extern relay_t relay;
