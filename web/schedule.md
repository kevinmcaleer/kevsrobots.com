---
layout: content
title: Show Schedule
subtitle: Join Kevin live for maker projects and robotics fun
description: Regular livestream schedule for KevsRobots - join us for live builds, Q&A sessions, and maker community hangouts
---

<style>
.calendar-row {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 0.5rem;
  margin: 2rem auto;
  max-width: 1200px;
}

@media (max-width: 768px) {
  .calendar-row {
    grid-template-columns: 1fr;
  }
}

.month-calendar {
  background: white;
  border: 1px solid #dee2e6;
  border-radius: 0.25rem;
  padding: 0.35rem;
  box-shadow: 0 1px 2px rgba(0,0,0,0.05);
}

.month-title {
  text-align: center;
  font-weight: bold;
  font-size: 0.85rem;
  margin-bottom: 0.25rem;
  color: #212529;
  padding-bottom: 0.25rem;
  border-bottom: 1px solid #e9ecef;
}

.mini-calendar-grid {
  display: grid;
  grid-template-columns: repeat(7, 1fr);
  gap: 1px;
}

.mini-calendar-header {
  text-align: center;
  font-weight: 600;
  font-size: 0.6rem;
  color: #6c757d;
  padding: 0.1rem 0;
}

.mini-calendar-day {
  aspect-ratio: 1;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 0.65rem;
  border-radius: 0.15rem;
  cursor: default;
  position: relative;
  padding: 0;
  margin: 0;
}

.mini-calendar-day.other-month {
  color: #ced4da;
  background: #fafafa;
}

.mini-calendar-day.current-month {
  background: white;
  color: #495057;
}

.mini-calendar-day.today {
  background: #e7f3ff;
  font-weight: bold;
  color: #0d6efd;
  border: 1px solid #0d6efd;
}

.mini-calendar-day.has-event {
  background: #dc3545;
  color: white;
  font-weight: bold;
  border-radius: 50%;
}

.mini-calendar-day.has-event:hover {
  transform: scale(1.1);
  transition: transform 0.1s ease;
  z-index: 10;
}

.event-list {
  max-width: 1200px;
  margin: 2rem auto;
}

.event-item {
  border-left: 4px solid #dc3545;
  padding: 0.5rem;
  margin-bottom: 0.5rem;
  background: white;
  border-radius: 0.25rem;
  box-shadow: 0 1px 3px rgba(0,0,0,0.1);
}

.event-item:hover {
  box-shadow: 0 2px 6px rgba(0,0,0,0.15);
  transform: translateX(2px);
  transition: all 0.2s ease;
}

.event-date {
  font-size: 0.875rem;
  color: #6c757d;
  margin-bottom: 0.5rem;
}

.event-title {
  font-size: 1.25rem;
  font-weight: bold;
  margin-bottom: 0.5rem;
  color: #212529;
}

.event-meta {
  font-size: 0.875rem;
  color: #6c757d;
}

.event-meta i {
  width: 1.25rem;
  text-align: center;
}

.legend {
  display: flex;
  gap: 1.5rem;
  justify-content: center;
  margin-top: 1.5rem;
  margin-bottom: 2rem;
  font-size: 0.875rem;
}

.legend-item {
  display: flex;
  align-items: center;
  gap: 0.5rem;
}

.legend-circle {
  width: 18px;
  height: 18px;
  border-radius: 50%;
  border: 1px solid #dee2e6;
}

.legend-circle.today {
  background: #e7f3ff;
  border-color: #0d6efd;
}

.legend-circle.event {
  background: #dc3545;
  border-color: #dc3545;
}
</style>

{% include nav_videos.html %}

{% include breadcrumbs.html %}

# Show Schedule
## Details of upcoming shows, calendar view, and how to subscribe to updates

This page provides a list of upcoming shows, a calendar view of scheduled events, and instructions on how to subscribe to the schedule so you never miss a live stream. All times are in the {{ site.site_timezone }} timezone.

---


{% comment %}Build event date lookup for quick checks{% endcomment %}
{% assign event_dates = "" | split: "" %}
{% for show in site.data.schedule %}
  {% if show.status == "active" %}
    {% for date in show.next_dates %}
      {% assign event_dates = event_dates | push: date %}
    {% endfor %}
  {% endif %}
{% endfor %}

## Upcoming Dates

<div class="calendar-row">
{% comment %}Generate 3 monthly calendars{% endcomment %}
{% for month_offset in (0..2) %}
  {% assign days_offset = month_offset | times: 30 | times: 86400 %}
  {% assign month_start = "now" | date: "%s" | plus: days_offset %}
  {% assign target_month = month_start | date: "%Y-%m-01" %}
  {% assign month_name = target_month | date: "%B %Y" %}
  {% assign first_day_of_month = target_month | date: "%w" %}
  {% assign days_in_month = target_month | date: "%Y-%m-01" | date: "%s" %}

  {% comment %}Calculate days in this month{% endcomment %}
  {% assign next_month = target_month | date: "%s" | plus: 2678400 | date: "%Y-%m-01" %}
  {% assign month_end = next_month | date: "%s" | minus: 86400 %}
  {% assign total_days = target_month | date: "%s" %}
  {% assign day_count = 1 %}
  {% for d in (1..31) %}
    {% assign check_date = target_month | date: "%Y-%m-" | append: d %}
    {% assign check_month = check_date | date: "%m" %}
    {% assign target_month_num = target_month | date: "%m" %}
    {% if check_month == target_month_num %}
      {% assign day_count = d %}
    {% endif %}
  {% endfor %}

  <div class="month-calendar">
    <div class="month-title">{{ month_name }}</div>
    <div class="mini-calendar-grid">
      <div class="mini-calendar-header">S</div>
      <div class="mini-calendar-header">M</div>
      <div class="mini-calendar-header">T</div>
      <div class="mini-calendar-header">W</div>
      <div class="mini-calendar-header">T</div>
      <div class="mini-calendar-header">F</div>
      <div class="mini-calendar-header">S</div>

      {% comment %}Add empty cells for days before month starts{% endcomment %}
      {% assign first_day_num = target_month | date: "%w" | times: 1 %}
      {% for i in (1..first_day_num) %}
        <div class="mini-calendar-day other-month"></div>
      {% endfor %}

      {% comment %}Add days of the month{% endcomment %}
      {% for day in (1..day_count) %}
        {% if day < 10 %}
          {% assign day_str = "0" | append: day %}
        {% else %}
          {% assign day_str = day | append: "" %}
        {% endif %}
        {% assign current_date = target_month | date: "%Y-%m-" | append: day_str %}
        {% assign today = "now" | date: "%Y-%m-%d" %}

        {% comment %}Check if this day has an event{% endcomment %}
        {% assign has_event = false %}
        {% if event_dates contains current_date %}
          {% assign has_event = true %}
        {% endif %}

        {% assign day_class = "mini-calendar-day current-month" %}
        {% if current_date == today %}
          {% assign day_class = day_class | append: " today" %}
        {% endif %}
        {% if has_event %}
          {% assign day_class = day_class | append: " has-event" %}
        {% endif %}

        <div class="{{ day_class }}" title="{{ current_date | date: '%B %-d, %Y' }}">
          {{ day }}
        </div>
      {% endfor %}

      {% comment %}Fill remaining cells{% endcomment %}
      {% assign total_cells = first_day_num | plus: day_count %}
      {% assign remaining = 42 | minus: total_cells %}
      {% if remaining > 7 %}
        {% assign remaining = remaining | minus: 7 %}
      {% endif %}
      {% for i in (1..remaining) %}
        <div class="mini-calendar-day other-month"></div>
      {% endfor %}
    </div>
  </div>
{% endfor %}
</div>

<div class="legend">
  <div class="legend-item">
    <div class="legend-circle today"></div>
    <span>Today</span>
  </div>
  <div class="legend-item">
    <div class="legend-circle event"></div>
    <span>Scheduled Show</span>
  </div>
</div>

---

## Upcoming Shows

<div class="event-list">

{% comment %}
Sort all events by date and display chronologically
{% endcomment %}
{% assign sorted_events = "" | split: "" %}
{% for show in site.data.schedule %}
  {% if show.status == "active" %}
    {% for date in show.next_dates %}
      {% assign date_timestamp = date | date: "%s" %}
      {% assign today_timestamp = "now" | date: "%s" %}
      {% if date_timestamp >= today_timestamp %}
        {% assign event_data = date | append: "|" | append: show.name | append: "|" | append: show.description | append: "|" | append: show.time | append: "|" | append: show.duration | append: "|" | append: show.category | append: "|" | append: show.youtube_channel %}
        {% assign sorted_events = sorted_events | push: event_data %}
      {% endif %}
    {% endfor %}
  {% endif %}
{% endfor %}

{% assign sorted_events = sorted_events | sort %}

{% for event_data in sorted_events limit:10 %}
  {% assign event_parts = event_data | split: "|" %}
  {% assign event_date = event_parts[0] %}
  {% assign event_name = event_parts[1] %}
  {% assign event_desc = event_parts[2] %}
  {% assign event_time = event_parts[3] %}
  {% assign event_duration = event_parts[4] %}
  {% assign event_category = event_parts[5] %}
  {% assign event_youtube = event_parts[6] %}

  <div class="event-item">
    <div class="event-date">
      <i class="fa-regular fa-calendar"></i>
      {{ event_date | date: "%A, %B %-d, %Y" }}
    </div>
    <div class="event-title">{{ event_name }}</div>
    <div class="mb-2">{{ event_desc }}</div>
    <div class="event-meta">
      <i class="fa-regular fa-clock"></i> {{ event_time }} ({{ site.site_timezone }})
      <span class="mx-2">•</span>
      <i class="fa-solid fa-hourglass-half"></i> {{ event_duration }}
      <span class="mx-2">•</span>
      <span class="badge bg-primary">{{ event_category }}</span>
      <span class="mx-2">•</span>
      <a href="{{ event_youtube }}" target="_blank"><i class="fa-brands fa-youtube"></i> Watch on YouTube</a>
    </div>
  </div>
{% endfor %}

</div>

---

## Regular Schedule

Quick reference for recurring shows:

{% for show in site.data.schedule %}
{% if show.status == "active" %}
<div class="card mb-3">
  <div class="card-body">
    <h5 class="card-title">{{ show.name }}</h5>
    <p class="card-text text-muted mb-2">{{ show.description }}</p>
    <p class="mb-1">
      <i class="fa-regular fa-calendar"></i> <strong>{{ show.frequency }}</strong>
      <span class="mx-2">•</span>
      <i class="fa-regular fa-clock"></i> {{ show.time }} ({{ site.site_timezone }})
      <span class="mx-2">•</span>
      <i class="fa-solid fa-hourglass-half"></i> {{ show.duration }}
    </p>
  </div>
</div>
{% endif %}
{% endfor %}

---

## Subscribe to Calendar

Never miss a show! Download the calendar file and import it into your favorite calendar app:

<div class="d-grid gap-2 d-md-block mb-4">
  <a href="/schedule.ics" class="btn btn-primary btn-lg" download>
    <i class="fa-solid fa-download"></i> Download Calendar (ICS)
  </a>
  <a href="https://www.youtube.com/@kevinmcaleer28" class="btn btn-danger btn-lg" target="_blank">
    <i class="fa-brands fa-youtube"></i> Subscribe on YouTube
  </a>
</div>

**How to import:**
- **Google Calendar:** Download the file, then Settings → Import & Export → Import
- **Apple Calendar:** Double-click the downloaded file
- **Outlook:** File → Open & Export → Import/Export → Import an iCalendar file

<div class="alert alert-info mt-4">
  <i class="fa-solid fa-bell"></i>
  <strong>Tip:</strong> Subscribe on YouTube and enable notifications to get alerts when shows go live!
</div>
