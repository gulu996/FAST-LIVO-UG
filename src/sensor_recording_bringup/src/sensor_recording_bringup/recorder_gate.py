def effective_required_topics(
    configured_topics,
    enable_livox=False,
    enable_camera=False,
    enable_gnss=False,
    enable_uwb=False,
):
    if configured_topics:
        return list(dict.fromkeys(configured_topics))
    topics = ["/sensor_time/events"]
    if enable_livox:
        topics.append("/livox/lidar")
    if enable_camera:
        topics.append("/left_camera/image")
    if enable_gnss:
        topics.append("/gnss/raw")
    if enable_uwb:
        topics.append("/uwb/raw")
    return topics


class RecorderGate:
    """Readiness depends on LOCAL/session and enabled topics, never UTC or GNSS fix."""

    def __init__(self, required_topics):
        self.required_topics = set(required_topics)
        self.seen_topics = set()
        self.session_id = 0
        self.local_ready = False

    def update_time(self, session_id, local_ready):
        changed = self.session_id not in (0, session_id) and session_id != 0
        self.session_id = session_id
        self.local_ready = local_ready
        return changed

    def mark_topic(self, topic):
        self.seen_topics.add(topic)

    def missing_topics(self):
        return sorted(self.required_topics - self.seen_topics)

    def status(self):
        if not self.local_ready:
            return "WAIT_LOCAL"
        if self.session_id == 0:
            return "WAIT_SESSION"
        missing = self.missing_topics()
        if missing:
            return "WAIT_TOPICS missing={}".format(",".join(missing))
        return "READY"

    def ready(self):
        return self.status() == "READY"
