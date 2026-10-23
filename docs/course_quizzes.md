# Course quizzes

Add a quick quiz anywhere in a course lesson (`source/<course>/*.md`) with a `{% quiz %}` block. Readers click an answer, get instant feedback, and their score for the course is shown in the lesson sidebar. Everything is stored in the reader's browser, so there's no backend and no login needed.

```markdown
## Check your understanding

{% quiz %}
? What does PWM stand for?
- [ ] Power Wave Mode
- [x] Pulse Width Modulation
- [ ] Pin Write Method
> PWM switches a pin on and off very fast to fake an analogue voltage.

? Which pin type can read an analogue voltage on the Pico?
- [ ] Any GPIO pin
- [x] ADC pins (GP26, GP27, GP28)
> Only the ADC pins are connected to the analogue-to-digital converter.
{% endquiz %}
```

| Line | Meaning |
| --- | --- |
| `? Question` | Starts a new question |
| `- [ ] Option` | A wrong answer |
| `- [x] Option` | A right answer (you can mark more than one) |
| `> Explanation` | Shown after the reader answers (optional) |

Text can use inline Markdown: `code`, **bold**, *italics* and [links](/learn/).

## How scoring works

- Only the reader's first answer to each question counts. They can press **Try again** to have another go, but the score doesn't change.
- `build.py` counts every question in the course and writes `quiz_total` into each lesson's front matter, so the sidebar can show "Quiz score 7 / 11". Run `python3 build.py` after adding quizzes.
- The sidebar progress bar counts the lessons the reader has actually opened, with a tick next to each one. "Reset my progress" clears it for that course.
- Code: `web/_plugins/quiz.rb` (renders the block), `web/assets/js/course-progress.js` (scoring and progress), styles under "Lesson quizzes" in `web/assets/css/main.scss`.

## Tips

- Two or three questions at the end of a lesson works well. Put them under a `## Check your understanding` heading.
- Make the wrong answers believable, and use the explanation to teach, not just to say "correct".
- Don't add a question if its answer can't be found in the lesson.
