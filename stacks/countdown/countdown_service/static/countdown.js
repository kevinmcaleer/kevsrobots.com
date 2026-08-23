const body = document.body;
const creatorView = document.querySelector("#creator-view");
const eventView = document.querySelector("#event-view");
const form = document.querySelector("#countdown-form");
const formError = document.querySelector("#form-error");
const dateInput = document.querySelector("#event-date-input");
const timeInput = document.querySelector("#event-time-input");
const descriptionInput = document.querySelector("#description-input");
const imageInput = document.querySelector("#image-input");
const imagePanel = document.querySelector("#image-panel");
const eventImage = document.querySelector("#event-image");
const eventDescription = document.querySelector("#event-description");
const eventDate = document.querySelector("#event-date");
const finishedMessage = document.querySelector("#finished-message");
const shareUrl = document.querySelector("#share-url");
const embedCode = document.querySelector("#embed-code");
const copyStatus = document.querySelector("#copy-status");

let timerId = null;

if (window.self !== window.top) {
  body.classList.add("is-embedded");
}

function pad(value) {
  return String(value).padStart(2, "0");
}

function localDateValue(date) {
  return `${date.getFullYear()}-${pad(date.getMonth() + 1)}-${pad(date.getDate())}`;
}

dateInput.min = localDateValue(new Date());

function eventUrl(target, description, image, dateOnly) {
  const url = new URL(window.location.pathname, window.location.origin);
  url.searchParams.set(
    "at",
    dateOnly ? localDateValue(target) : target.toISOString(),
  );
  if (description) {
    url.searchParams.set("description", description);
  }
  if (image) {
    url.searchParams.set("image", image);
  }
  return url;
}

function escapedAttribute(value) {
  return value
    .replaceAll("&", "&amp;")
    .replaceAll('"', "&quot;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;");
}

function updateShareDetails(url, description) {
  const urlText = url.toString();
  const title = description || "Event countdown";
  shareUrl.value = urlText;
  embedCode.value =
    `<iframe src="${escapedAttribute(urlText)}" ` +
    `title="${escapedAttribute(title)}" width="100%" height="600" ` +
    'style="border:0;border-radius:16px" loading="lazy"></iframe>';
}

function daysInMonth(year, month) {
  return new Date(year, month + 1, 0).getDate();
}

function addCalendarMonths(date, months) {
  const result = new Date(date.getFullYear(), date.getMonth() + months, 1);
  result.setDate(
    Math.min(date.getDate(), daysInMonth(result.getFullYear(), result.getMonth())),
  );
  return result;
}

function calendarDaysBetween(start, end) {
  const startUtc = Date.UTC(start.getFullYear(), start.getMonth(), start.getDate());
  const endUtc = Date.UTC(end.getFullYear(), end.getMonth(), end.getDate());
  return Math.floor((endUtc - startUtc) / 86400000);
}

function setCountdownUnits(values, units, padValues) {
  const valueElements = ["days", "hours", "minutes", "seconds"];
  valueElements.forEach((id, index) => {
    const value = values[index];
    document.querySelector(`#${id}`).textContent =
      padValues ? pad(value) : String(value);
    document.querySelector(`#${id}-label`).textContent =
      value === 1 ? units[index].slice(0, -1) : units[index];
  });
}

function renderDateCountdown(target) {
  const today = new Date();
  const cursorStart = new Date(
    today.getFullYear(),
    today.getMonth(),
    today.getDate(),
  );
  if (target <= cursorStart) {
    setCountdownUnits([0, 0, 0, 0], ["years", "months", "weeks", "days"], false);
    return true;
  }

  let years = target.getFullYear() - cursorStart.getFullYear();
  let cursor = addCalendarMonths(cursorStart, years * 12);
  if (cursor > target) {
    years -= 1;
    cursor = addCalendarMonths(cursorStart, years * 12);
  }

  let months =
    (target.getFullYear() - cursor.getFullYear()) * 12 +
    target.getMonth() -
    cursor.getMonth();
  let monthCursor = addCalendarMonths(cursor, months);
  if (monthCursor > target) {
    months -= 1;
    monthCursor = addCalendarMonths(cursor, months);
  }

  const remainingDays = calendarDaysBetween(monthCursor, target);
  const weeks = Math.floor(remainingDays / 7);
  const days = remainingDays % 7;
  setCountdownUnits(
    [years, months, weeks, days],
    ["years", "months", "weeks", "days"],
    false,
  );
  return false;
}

function renderCountdown(target, dateOnly) {
  if (dateOnly) {
    const finished = renderDateCountdown(target);
    finishedMessage.hidden = !finished;
    if (finished && timerId !== null) {
      window.clearInterval(timerId);
      timerId = null;
    }
    return;
  }

  const remaining = Math.max(0, target.getTime() - Date.now());
  const totalSeconds = Math.floor(remaining / 1000);
  const days = Math.floor(totalSeconds / 86400);
  const hours = Math.floor((totalSeconds % 86400) / 3600);
  const minutes = Math.floor((totalSeconds % 3600) / 60);
  const seconds = totalSeconds % 60;

  setCountdownUnits(
    [days, hours, minutes, seconds],
    ["days", "hours", "minutes", "seconds"],
    true,
  );

  if (remaining === 0) {
    finishedMessage.hidden = false;
    if (timerId !== null) {
      window.clearInterval(timerId);
      timerId = null;
    }
  }
}

function showEvent(target, description, image, dateOnly) {
  if (timerId !== null) {
    window.clearInterval(timerId);
  }

  creatorView.hidden = true;
  eventView.hidden = false;
  eventDescription.textContent = description || "Your event";
  const dateFormat = { dateStyle: "full" };
  if (!dateOnly) {
    dateFormat.timeStyle = "short";
  }
  eventDate.textContent = new Intl.DateTimeFormat(undefined, dateFormat).format(target);

  imagePanel.hidden = !image;
  eventView.classList.toggle("has-image", Boolean(image));
  if (image) {
    eventImage.src = image;
    eventImage.alt = description ? `Image for ${description}` : "Countdown event";
  } else {
    eventImage.removeAttribute("src");
  }

  finishedMessage.hidden = true;
  const url = eventUrl(target, description, image, dateOnly);
  updateShareDetails(url, description);
  window.history.replaceState({}, "", url);
  renderCountdown(target, dateOnly);
  timerId = window.setInterval(
    () => renderCountdown(target, dateOnly),
    dateOnly ? 60000 : 1000,
  );
}

eventImage.addEventListener("error", () => {
  imagePanel.hidden = true;
  eventView.classList.remove("has-image");
});

form.addEventListener("submit", (event) => {
  event.preventDefault();
  formError.hidden = true;

  if (!dateInput.value) {
    formError.textContent = "Choose a date for your countdown.";
    formError.hidden = false;
    return;
  }

  const [year, month, day] = dateInput.value.split("-").map(Number);
  const dateOnly = !timeInput.value;
  const [hour, minute] = (timeInput.value || "00:00").split(":").map(Number);
  const target = new Date(year, month - 1, day, hour, minute);

  if (Number.isNaN(target.getTime())) {
    formError.textContent = "Choose a valid date and time.";
    formError.hidden = false;
    return;
  }

  showEvent(
    target,
    descriptionInput.value.trim(),
    imageInput.value.trim(),
    dateOnly,
  );
});

document.querySelector("#new-countdown").addEventListener("click", () => {
  if (timerId !== null) {
    window.clearInterval(timerId);
    timerId = null;
  }
  eventView.hidden = true;
  creatorView.hidden = false;
  window.history.replaceState({}, "", window.location.pathname);
  dateInput.focus();
});

document.querySelectorAll("[data-copy-target]").forEach((button) => {
  button.addEventListener("click", async () => {
    const target = document.querySelector(`#${button.dataset.copyTarget}`);
    try {
      await navigator.clipboard.writeText(target.value);
      copyStatus.textContent = "Copied to clipboard.";
    } catch {
      target.select();
      copyStatus.textContent = "Select the text and copy it manually.";
    }
  });
});

const initialAt = body.dataset.at;
if (initialAt) {
  const dateOnly = body.dataset.dateOnly === "true";
  let target;
  if (dateOnly && /^\d{4}-\d{2}-\d{2}$/.test(initialAt)) {
    const [year, month, day] = initialAt.split("-").map(Number);
    target = new Date(year, month - 1, day);
  } else {
    target = new Date(initialAt);
  }
  if (!Number.isNaN(target.getTime())) {
    const description = body.dataset.description || "";
    const image = body.dataset.image || "";
    descriptionInput.value = description;
    imageInput.value = image;
    dateInput.value = localDateValue(target);
    timeInput.value = dateOnly
      ? ""
      : `${pad(target.getHours())}:${pad(target.getMinutes())}`;
    showEvent(target, description, image, dateOnly);
  }
}

if (body.dataset.error) {
  formError.textContent = body.dataset.error;
  formError.hidden = false;
}
