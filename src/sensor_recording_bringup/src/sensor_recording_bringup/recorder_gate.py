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

    def ready(self):
        return (
            self.local_ready
            and self.session_id != 0
            and self.required_topics.issubset(self.seen_topics)
        )
