#include "events.h"

void events_t::add_event(string header, string msg) {
    event_list.push_back(header + ": " + msg);
}

void events_t::clear_events() {
    event_list.clear();
}

events_t *event_mgr = nullptr;