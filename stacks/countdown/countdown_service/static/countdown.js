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
  url.searchParams.set("at", target.toISOString());
  if (dateOnly) {
    url.searchParams.set("date_only", "1");
  }
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

function renderCountdown(target, dateOnly) {
  const remaining = Math.max(0, target.getTime() - Date.now());
  const totalSeconds = Math.floor(remaining / 1000);
  const days = dateOnly
    ? Math.ceil(remaining / 86400000)
    : Math.floor(totalSeconds / 86400);
  const hours = Math.floor((totalSeconds % 86400) / 3600);
  const minutes = Math.floor((totalSeconds % 3600) / 60);
  const seconds = totalSeconds % 60;

  document.querySelector("#days").textContent = String(days);
  document.querySelector("#hours").textContent = pad(hours);
  document.querySelector("#minutes").textContent = pad(minutes);
  document.querySelector("#seconds").textContent = pad(seconds);

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
  document.querySelector("#countdown").classList.toggle("days-only", dateOnly);

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
  timerId = window.setInterval(() => renderCountdown(target, dateOnly), 1000);
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
  const target = new Date(initialAt);
  if (!Number.isNaN(target.getTime())) {
    const description = body.dataset.description || "";
    const image = body.dataset.image || "";
    const dateOnly = body.dataset.dateOnly === "true";
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
