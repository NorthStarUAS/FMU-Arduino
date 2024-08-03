#include <string>
#include <vector>
using std::string;
using std::vector;

class events_t {

public:

    vector<string> event_list;
    void add_event(string header, string msg);
    void clear_events();

};

extern events_t *event_mgr;