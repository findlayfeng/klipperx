//! `objects/subscribe` — [`objects/query`](super::objects_query) on a timer,
//! pushing what changed.
//!
//! ```json
//! {"id": 1, "method": "objects/subscribe",
//!  "params": {"objects": {"toolhead": ["position"]},
//!             "response_template": {"method": "printer:status", "id": null}}}
//! ```
//!
//! The request is answered **immediately with a full snapshot**, exactly as
//! `objects/query` would — every requested field, not only what changed. After
//! that the connection is registered, and while it stays open the host pushes a
//! message every [`SUBSCRIPTION_REFRESH_TIME`] (0.25 s) carrying only the fields
//! whose values changed since the last push:
//!
//! ```json
//! {"method": "printer:status", "id": null,
//!  "params": {"eventtime": 12.5, "status": {"toolhead": {"position": [...]}}}}
//! ```
//!
//! The pushed message is the client's `response_template` with `params` filled
//! in, which is [`ResponseTemplate`]'s job — the same merge upstream does.
//!
//! # Upstream
//!
//! `QueryStatusHelper` (`klippy/webhooks.py:471-562`) serves `objects/list`,
//! `objects/query` and `objects/subscribe` from one timer. Two of its decisions
//! are visible to a client and kept here:
//!
//! * a **`null` field list becomes the object's fields as they are**, and only
//!   fields that were there are tracked — a field an object grows later is not
//!   pushed, because the client never asked for it by name;
//! * a **`null`-valued field is not a change**: a field that stays absent is
//!   not pushed, but a field that appears or disappears is.
//!
//! Two things differ, both because this host has no pending-query tick:
//!
//! * the reply is sent **at request time** rather than at the next timer tick,
//!   so a subscription never waits for the 0.25 s;
//! * changes are measured against **what that client last saw**, not against a
//!   snapshot shared by every client. In steady state the two are the same; they
//!   differ only for a client that subscribes while others are already running,
//!   and this way that client cannot be sent a change that happened before it
//!   subscribed.
//!
//! # Lifetime
//!
//! Cancelling is **closing the connection** — there is no unsubscribe request,
//! upstream has none either. A closed connection is dropped at the next tick,
//! and when the last one goes the timer retires itself. Re-subscribing on the
//! same connection replaces its subscription.

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex, MutexGuard};

use serde_json::{json, Map, Value};

use super::objects_query::{select_fields, status_object, ObjectsQueryParams};
use crate::core::klippy::api::protocol::{PushTarget, Request, ResponseTemplate};
use crate::core::klippy::api::registry::{Endpoint, EndpointContext, EndpointFuture};
use crate::core::klippy::api::{Api, ApiWiring, RegistrationError};
use crate::core::klippy::printer::Printer;
use crate::core::klippy::reactor::{Reactor, TimerHandle};

endpoint!(install);

/// Install the `objects/subscribe` endpoint.
pub(crate) fn install(api: &mut Api, wiring: &ApiWiring<'_>) -> Result<(), RegistrationError> {
    api.register(ObjectsSubscribe::new(Arc::clone(wiring.printer)))
        .map_err(RegistrationError::Endpoint)
}

/// Seconds between subscription refreshes: upstream's
/// `SUBSCRIPTION_REFRESH_TIME` (`klippy/webhooks.py:469`).
pub const SUBSCRIPTION_REFRESH_TIME: f64 = 0.25;

/// The `objects/subscribe` endpoint.
///
/// One endpoint serves every client: the subscriptions and the single refresh
/// timer they share live behind the state mutex, not in the registry.
pub struct ObjectsSubscribe {
    inner: Arc<Inner>,
}

/// The endpoint's state, shared with its timer callback.
struct Inner {
    printer: Arc<Printer>,
    reactor: Arc<dyn Reactor>,
    state: Mutex<State>,
}

struct State {
    clients: Vec<Subscription>,
    /// The refresh timer while at least one client is subscribed.
    timer: Option<TimerHandle>,
}

/// One connected client's subscription.
struct Subscription {
    client: Arc<dyn PushTarget>,
    /// Object name to the fields tracked, or `None` while the object reports
    /// nothing — the field list is taken from the first tick that sees it, so a
    /// `null` request for an object that appears later still starts working.
    fields: BTreeMap<String, Option<Vec<String>>>,
    /// Object name to the values last reported to this client. What a tick
    /// compares against, so it pushes changes and not repeats.
    last: BTreeMap<String, Map<String, Value>>,
    template: ResponseTemplate,
}

impl ObjectsSubscribe {
    /// Build the endpoint over the machine it watches.
    pub fn new(printer: Arc<Printer>) -> Self {
        let reactor = printer.reactor();
        Self {
            inner: Arc::new(Inner {
                printer,
                reactor,
                state: Mutex::new(State {
                    clients: Vec::new(),
                    timer: None,
                }),
            }),
        }
    }

    /// Replace this connection's subscription with a fresh one.
    fn subscribe(
        &self,
        client: Arc<dyn PushTarget>,
        fields: BTreeMap<String, Option<Vec<String>>>,
        last: BTreeMap<String, Map<String, Value>>,
        template: ResponseTemplate,
    ) {
        let mut state = self.lock();
        // Upstream deletes and re-adds a re-subscribing connection, so the
        // second request replaces the first rather than adding a second push
        // per tick.
        state
            .clients
            .retain(|sub| !Arc::ptr_eq(&sub.client, &client));
        state.clients.push(Subscription {
            client,
            fields,
            last,
            template,
        });
    }

    /// Start the refresh timer if it is not running.
    ///
    /// Reads the clock now rather than at the next tick so that the reply the
    /// client already got is not repeated as the first push.
    fn ensure_timer(&self) {
        let mut state = self.lock();
        if state.timer.is_some() {
            return;
        }
        let inner = Arc::clone(&self.inner);
        let handle = self.inner.reactor.register_timer_named(
            "objects/subscribe",
            Box::new(move |eventtime| inner.tick(eventtime)),
            self.inner.reactor.monotonic() + SUBSCRIPTION_REFRESH_TIME,
        );
        state.timer = Some(handle);
    }

    fn lock(&self) -> MutexGuard<'_, State> {
        self.inner
            .state
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

impl Endpoint for ObjectsSubscribe {
    fn path(&self) -> &'static str {
        "objects/subscribe"
    }

    fn handle<'a>(
        &'a self,
        request: &'a Request,
        context: &'a EndpointContext<'a>,
    ) -> EndpointFuture<'a> {
        Box::pin(async move {
            let params = ObjectsQueryParams::from_request(request)?;
            let template = ResponseTemplate::from_params(&request.params())?;

            let (reply, fields, last) = snapshot(&self.inner.printer, &params);
            self.subscribe(context.client.clone(), fields, last, template);
            self.ensure_timer();

            Ok(reply)
        })
    }
}

impl Inner {
    /// One refresh: push every subscription what changed, retire the timer when
    /// nobody is left.
    ///
    /// This is the reactor callback, so it runs on the reactor's thread and must
    /// not block. `get_status` is called at most once per object per tick and
    /// shared by every subscriber, which is upstream's `query` cache.
    fn tick(&self, eventtime: f64) -> Option<f64> {
        let mut state = self
            .state
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());

        // A subscription lives as long as its connection.
        state.clients.retain(|sub| !sub.client.is_closed());
        if state.clients.is_empty() {
            // Nobody left: returning `None` retires the timer, and dropping the
            // handle here means the next subscription starts a new one.
            state.timer = None;
            return None;
        }

        let mut cache: BTreeMap<String, Map<String, Value>> = BTreeMap::new();
        for subscription in state.clients.iter_mut() {
            let status = subscription.changed(&self.printer, eventtime, &mut cache);
            if !status.is_empty() {
                let params = json!({ "eventtime": eventtime, "status": status });
                subscription
                    .client
                    .push(subscription.template.message(params));
            }
        }

        Some(eventtime + SUBSCRIPTION_REFRESH_TIME)
    }
}

impl Subscription {
    /// The fields that changed since this client was last pushed, as a status
    /// object ready to send. Empty when nothing changed.
    fn changed(
        &mut self,
        printer: &Printer,
        eventtime: f64,
        cache: &mut BTreeMap<String, Map<String, Value>>,
    ) -> Map<String, Value> {
        let mut status = Map::new();
        // The field lists may be extended below (a `null` request that only now
        // found something to track), so the names are taken first.
        let names: Vec<String> = self.fields.keys().cloned().collect();

        for name in names {
            let full = cache
                .entry(name.clone())
                .or_insert_with(|| status_object(printer, &name, eventtime));

            // A `null` request is expanded the first time the object reports
            // anything; until then there is nothing to track, and once expanded
            // it is a fixed list (upstream's `subscription[obj] = req_items`).
            let tracked: Vec<String> = match self.fields.get(&name) {
                Some(Some(fields)) => fields.clone(),
                _ => {
                    let fields: Vec<String> = full.keys().cloned().collect();
                    if !fields.is_empty() {
                        self.fields.insert(name.clone(), Some(fields.clone()));
                    }
                    fields
                }
            };

            let last = self.last.entry(name.clone()).or_default();
            let mut changed = Map::new();
            for field in tracked {
                let now = full.get(&field).cloned().unwrap_or(Value::Null);
                // `null` is what a field that is not there looks like, so a
                // field that stays absent is not a change and one that appears
                // or disappears is.
                if now != last.get(&field).cloned().unwrap_or(Value::Null) {
                    changed.insert(field.clone(), now.clone());
                    last.insert(field, now);
                }
            }
            if !changed.is_empty() {
                status.insert(name, Value::Object(changed));
            }
        }
        status
    }
}

/// The immediate reply, and the state the subscription tracks from.
///
/// The reply is a full query: every requested field, `null` or not. The field
/// lists and last values are what the timer compares against, and they are read
/// in the same pass so the reply and the baseline cannot disagree.
#[allow(clippy::type_complexity)]
fn snapshot(
    printer: &Printer,
    params: &ObjectsQueryParams,
) -> (
    Value,
    BTreeMap<String, Option<Vec<String>>>,
    BTreeMap<String, Map<String, Value>>,
) {
    let eventtime = printer.eventtime();
    let mut status = Map::new();
    let mut fields = BTreeMap::new();
    let mut last = BTreeMap::new();

    for (name, requested) in params.objects() {
        let full = status_object(printer, name, eventtime);
        status.insert(name.clone(), select_fields(full.clone(), requested));

        let tracked: Option<Vec<String>> = match requested {
            // An object that reports nothing yet has no fields to name; the
            // list stays open until a tick finds some.
            Value::Null if full.is_empty() => None,
            Value::Null => Some(full.keys().cloned().collect()),
            Value::Array(requested) => Some(
                requested
                    .iter()
                    .map(|field| field.as_str().expect("validated to be strings").to_string())
                    .collect(),
            ),
            _ => unreachable!("validated in from_params"),
        };
        let values = match &tracked {
            Some(names) => names
                .iter()
                .map(|field| {
                    (
                        field.clone(),
                        full.get(field).cloned().unwrap_or(Value::Null),
                    )
                })
                .collect(),
            None => Map::new(),
        };
        fields.insert(name.clone(), tracked);
        last.insert(name.clone(), values);
    }

    (
        json!({ "eventtime": eventtime, "status": status }),
        fields,
        last,
    )
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::api::protocol::ApiError;
    use crate::core::klippy::api::registry::Api;
    use crate::core::klippy::api::test_support::{context, MutableStatus, RecordingTarget};
    use crate::core::klippy::reactor::ManualReactor;

    fn request(body: &str) -> Request {
        Request::parse(body.as_bytes()).expect("test body is a valid request")
    }

    /// An endpoint over a machine with a `toolhead` the test can change.
    ///
    /// Returns the object itself, so a test writes a status directly instead of
    /// going through the printer.
    fn endpoint() -> (ObjectsSubscribe, Arc<ManualReactor>, Arc<MutableStatus>) {
        let reactor = Arc::new(ManualReactor::new());
        let printer = Arc::new(Printer::new(reactor.clone()));
        let toolhead = Arc::new(MutableStatus::new(json!({
            "position": [20.0, 30.0, 5.0, 0.0],
            "max_velocity": 300.0,
        })));
        printer.add_object("toolhead", toolhead.clone()).unwrap();
        (ObjectsSubscribe::new(printer), reactor, toolhead)
    }

    /// Subscribe `objects` on `target` and return the immediate reply.
    async fn subscribe(
        endpoint: &ObjectsSubscribe,
        api: &Api,
        target: Arc<RecordingTarget>,
        objects: &str,
    ) -> Value {
        let body = format!(r#"{{"method":"objects/subscribe","params":{{"objects":{objects}}}}}"#);
        endpoint
            .handle(&request(&body), &context(api, target))
            .await
            .unwrap()
    }

    /// What the only push carried, as the `status` object.
    fn pushed_status(target: &RecordingTarget) -> Value {
        target.pushes()[0]["params"]["status"].clone()
    }

    #[tokio::test]
    async fn test_the_endpoint_path_is_the_documented_one() {
        let (endpoint, _reactor, _toolhead) = endpoint();
        assert_eq!(endpoint.path(), "objects/subscribe");
    }

    #[tokio::test]
    async fn test_the_request_is_answered_with_a_full_snapshot() {
        let (endpoint, _reactor, _toolhead) = endpoint();
        let api = Api::new();
        let target = RecordingTarget::new();

        let reply = subscribe(
            &endpoint,
            &api,
            target.clone(),
            r#"{"toolhead":["position","max_velocity"]}"#,
        )
        .await;

        assert_eq!(
            reply["status"]["toolhead"],
            json!({"position": [20.0, 30.0, 5.0, 0.0], "max_velocity": 300.0})
        );
        assert!(reply["eventtime"].as_f64().is_some());
        // Nothing is pushed before the first period.
        assert!(target.pushes().is_empty());
    }

    #[tokio::test]
    async fn test_a_change_is_pushed_after_a_period() {
        let (endpoint, reactor, toolhead) = endpoint();
        let api = Api::new();
        let target = RecordingTarget::new();
        subscribe(
            &endpoint,
            &api,
            target.clone(),
            r#"{"toolhead":["position"]}"#,
        )
        .await;

        toolhead.set(json!({
            "position": [21.0, 30.0, 5.0, 0.0],
            "max_velocity": 300.0,
        }));
        reactor.advance(SUBSCRIPTION_REFRESH_TIME);

        assert_eq!(
            target.pushes(),
            vec![json!({
                "params": {
                    "eventtime": SUBSCRIPTION_REFRESH_TIME,
                    "status": {"toolhead": {"position": [21.0, 30.0, 5.0, 0.0]}}
                }
            })]
        );
    }

    #[tokio::test]
    async fn test_only_changed_fields_are_pushed() {
        let (endpoint, reactor, toolhead) = endpoint();
        let api = Api::new();
        let target = RecordingTarget::new();
        subscribe(
            &endpoint,
            &api,
            target.clone(),
            r#"{"toolhead":["position","max_velocity"]}"#,
        )
        .await;

        toolhead.set(json!({
            "position": [1.0, 2.0, 3.0, 4.0],
            "max_velocity": 300.0,
        }));
        reactor.advance(SUBSCRIPTION_REFRESH_TIME);

        assert_eq!(
            pushed_status(&target)["toolhead"],
            json!({"position": [1.0, 2.0, 3.0, 4.0]})
        );
    }

    #[tokio::test]
    async fn test_an_unchanged_tick_pushes_nothing() {
        let (endpoint, reactor, _toolhead) = endpoint();
        let api = Api::new();
        let target = RecordingTarget::new();
        subscribe(
            &endpoint,
            &api,
            target.clone(),
            r#"{"toolhead":["position"]}"#,
        )
        .await;

        reactor.advance(SUBSCRIPTION_REFRESH_TIME);
        reactor.advance(SUBSCRIPTION_REFRESH_TIME);

        assert!(target.pushes().is_empty());
    }

    #[tokio::test]
    async fn test_the_response_template_wraps_every_push() {
        let (endpoint, reactor, toolhead) = endpoint();
        let api = Api::new();
        let target = RecordingTarget::new();
        let body = r#"{"method":"objects/subscribe","params":{"objects":{"toolhead":["position"]},"response_template":{"method":"printer:status","id":null}}}"#;
        endpoint
            .handle(&request(body), &context(&api, target.clone()))
            .await
            .unwrap();

        toolhead.set(json!({"position": [1.0, 2.0, 3.0, 4.0], "max_velocity": 300.0}));
        reactor.advance(SUBSCRIPTION_REFRESH_TIME);

        assert_eq!(target.pushes()[0]["method"], "printer:status");
        assert_eq!(target.pushes()[0]["id"], Value::Null);
        assert!(target.pushes()[0]["params"]["status"].is_object());
    }

    #[tokio::test]
    async fn test_a_null_field_list_tracks_the_fields_that_are_there() {
        let (endpoint, reactor, toolhead) = endpoint();
        let api = Api::new();
        let target = RecordingTarget::new();
        subscribe(&endpoint, &api, target.clone(), r#"{"toolhead":null}"#).await;

        toolhead.set(json!({
            "position": [9.0, 9.0, 9.0, 9.0],
            "max_velocity": 300.0,
        }));
        reactor.advance(SUBSCRIPTION_REFRESH_TIME);

        // `position` changed; `max_velocity` did not, so only `position` is sent.
        assert_eq!(
            pushed_status(&target)["toolhead"],
            json!({"position": [9.0, 9.0, 9.0, 9.0]})
        );
    }

    #[tokio::test]
    async fn test_a_field_that_appears_is_pushed_and_one_that_stays_absent_is_not() {
        let (endpoint, reactor, toolhead) = endpoint();
        let api = Api::new();
        let target = RecordingTarget::new();
        subscribe(&endpoint, &api, target.clone(), r#"{"toolhead":["nope"]}"#).await;

        // Still absent: `null` to `null` is not a change.
        reactor.advance(SUBSCRIPTION_REFRESH_TIME);
        assert!(target.pushes().is_empty());

        // Now it is there.
        toolhead.set(json!({"position": [1.0, 2.0, 3.0, 4.0], "nope": 5}));
        reactor.advance(SUBSCRIPTION_REFRESH_TIME);
        assert_eq!(pushed_status(&target)["toolhead"], json!({"nope": 5}));
    }

    #[tokio::test]
    async fn test_an_unknown_object_answers_empty_and_pushes_nothing() {
        let (endpoint, reactor, _toolhead) = endpoint();
        let api = Api::new();
        let target = RecordingTarget::new();

        let reply = subscribe(&endpoint, &api, target.clone(), r#"{"nope":["a"]}"#).await;
        assert_eq!(reply["status"]["nope"], json!({"a": null}));

        reactor.advance(SUBSCRIPTION_REFRESH_TIME);
        assert!(target.pushes().is_empty());
    }

    #[tokio::test]
    async fn test_a_closed_connection_is_dropped_and_the_timer_retires() {
        let (endpoint, reactor, toolhead) = endpoint();
        let api = Api::new();
        let target = RecordingTarget::new();
        subscribe(
            &endpoint,
            &api,
            target.clone(),
            r#"{"toolhead":["position"]}"#,
        )
        .await;

        target.close();
        toolhead.set(json!({"position": [1.0, 2.0, 3.0, 4.0]}));
        // One tick runs (and finds nothing to push), then the timer is gone.
        assert_eq!(reactor.advance(SUBSCRIPTION_REFRESH_TIME), 1);
        assert!(target.pushes().is_empty());
        assert_eq!(reactor.advance(SUBSCRIPTION_REFRESH_TIME), 0);
    }

    #[tokio::test]
    async fn test_re_subscribing_replaces_the_subscription() {
        let (endpoint, reactor, toolhead) = endpoint();
        let api = Api::new();
        let target = RecordingTarget::new();
        subscribe(
            &endpoint,
            &api,
            target.clone(),
            r#"{"toolhead":["position"]}"#,
        )
        .await;
        subscribe(
            &endpoint,
            &api,
            target.clone(),
            r#"{"toolhead":["max_velocity"]}"#,
        )
        .await;

        toolhead.set(json!({
            "position": [1.0, 2.0, 3.0, 4.0],
            "max_velocity": 301.0,
        }));
        reactor.advance(SUBSCRIPTION_REFRESH_TIME);

        // One push, tracking only what the second request asked for.
        assert_eq!(target.pushes().len(), 1);
        assert_eq!(
            pushed_status(&target)["toolhead"],
            json!({"max_velocity": 301.0})
        );
    }

    #[tokio::test]
    async fn test_one_timer_serves_every_subscriber() {
        let (endpoint, reactor, toolhead) = endpoint();
        let api = Api::new();
        let first = RecordingTarget::new();
        let second = RecordingTarget::new();
        subscribe(
            &endpoint,
            &api,
            first.clone(),
            r#"{"toolhead":["position"]}"#,
        )
        .await;
        subscribe(
            &endpoint,
            &api,
            second.clone(),
            r#"{"toolhead":["position"]}"#,
        )
        .await;

        toolhead.set(json!({"position": [1.0, 2.0, 3.0, 4.0]}));
        // A single tick, shared by both.
        assert_eq!(reactor.advance(SUBSCRIPTION_REFRESH_TIME), 1);

        assert_eq!(first.pushes().len(), 1);
        assert_eq!(second.pushes().len(), 1);
    }

    #[tokio::test]
    async fn test_the_registry_reaches_the_endpoint_by_path() {
        let (endpoint, reactor, toolhead) = endpoint();
        let mut api = Api::new();
        api.register(endpoint).unwrap();
        let target = RecordingTarget::new();
        let body =
            r#"{"method":"objects/subscribe","params":{"objects":{"toolhead":["position"]}}}"#;

        let reply = api.dispatch(&request(body), target.clone()).await.unwrap();

        assert!(reply["status"]["toolhead"].is_object());
        toolhead.set(json!({"position": [1.0, 2.0, 3.0, 4.0]}));
        reactor.advance(SUBSCRIPTION_REFRESH_TIME);
        assert_eq!(target.pushes().len(), 1);
    }

    #[tokio::test]
    async fn test_objects_is_required_and_validated_like_a_query() {
        let (endpoint, _reactor, _toolhead) = endpoint();
        let api = Api::new();
        let target = RecordingTarget::new();

        for (body, expected) in [
            (
                r#"{"method":"objects/subscribe"}"#,
                ApiError::MissingArgument("objects".to_string()),
            ),
            (
                r#"{"method":"objects/subscribe","params":{"objects":["toolhead"]}}"#,
                ApiError::InvalidArgumentType("objects".to_string()),
            ),
            (
                r#"{"method":"objects/subscribe","params":{"objects":{"toolhead":3}}}"#,
                ApiError::InvalidArgument,
            ),
            (
                r#"{"method":"objects/subscribe","params":{"objects":{"toolhead":["a",3]}}}"#,
                ApiError::InvalidArgument,
            ),
        ] {
            let error = endpoint
                .handle(&request(body), &context(&api, target.clone()))
                .await
                .unwrap_err();
            assert_eq!(error, expected, "{body}");
        }
    }

    #[tokio::test]
    async fn test_a_response_template_that_is_not_an_object_is_rejected() {
        let (endpoint, _reactor, _toolhead) = endpoint();
        let api = Api::new();
        let target = RecordingTarget::new();
        let body = r#"{"method":"objects/subscribe","params":{"objects":{"toolhead":["position"]},"response_template":7}}"#;

        let error = endpoint
            .handle(&request(body), &context(&api, target))
            .await
            .unwrap_err();

        assert_eq!(
            error,
            ApiError::InvalidArgumentType("response_template".to_string())
        );
    }
}
