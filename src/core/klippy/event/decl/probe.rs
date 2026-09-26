//! `probe:` events.

// The probe fires this after each probing move so a consumer can adjust the
// reported result in place (`axis_twist_compensation`, `probe.py:329`): the
// shared handle lets the sender read back whatever the handlers wrote.
event!("probe:update_results", { results: ProbeResultsHandle });
