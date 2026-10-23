# Quiz block for course lessons.
#
#   {% quiz %}
#   ? What does PWM stand for?
#   - [ ] Power Wave Mode
#   - [x] Pulse Width Modulation
#   > PWM switches a pin on and off very fast to fake an analogue voltage.
#   {% endquiz %}
#
# A block can hold several questions: each starts with a `?` line, followed by
# its options (`- [x]` marks a correct one) and an optional `>` explanation.
# Question, option and explanation text is inline Markdown. Scoring and
# progress live in assets/js/course-progress.js; styles in assets/css/main.scss.
# See docs/course_quizzes.md for the cheat sheet.

require "cgi"
require "kramdown"

module KevsRobots
  class QuizBlock < Liquid::Block
    def render(context)
      questions = parse_questions(super)
      return "" if questions.empty?

      page_url = context.registers[:page] && context.registers[:page]["url"]
      counts = (context.registers[:kr_quiz_counts] ||= Hash.new(0))

      html = +"\n<div class=\"kr-quiz\" data-quiz>\n"
      questions.each_with_index do |q, n|
        counts[page_url] += 1
        qid = "#{page_url}#q#{counts[page_url]}"
        answers = q[:options].each_index.select { |i| q[:options][i][:correct] }
        html << "<div class=\"kr-quiz-q\" data-qid=\"#{h(qid)}\" data-answer=\"#{answers.join(',')}\">"
        html << "<p class=\"kr-quiz-label\"><i class=\"fa-solid fa-circle-question\"></i> Quick quiz#{questions.size > 1 ? " #{n + 1} of #{questions.size}" : ''}</p>"
        html << "<p class=\"kr-quiz-question\">#{inline(q[:question])}</p>"
        html << "<div class=\"kr-quiz-options\">"
        q[:options].each_with_index do |opt, i|
          html << "<button type=\"button\" class=\"kr-quiz-option\" data-index=\"#{i}\">#{inline(opt[:text])}</button>"
        end
        html << "</div>"
        html << "<p class=\"kr-quiz-feedback\" aria-live=\"polite\"></p>"
        html << "<div class=\"kr-quiz-explain\" hidden>#{inline(q[:explain].join(' '))}</div>" unless q[:explain].empty?
        html << "</div>\n"
      end
      html << "</div>\n"
    end

    private

    def parse_questions(text)
      questions = []
      text.each_line do |raw|
        line = raw.strip
        next if line.empty?

        if line.start_with?("?")
          questions << { question: line[1..].strip, options: [], explain: [] }
        elsif (m = line.match(/\A[-*]\s*\[( |x|X)\]\s*(.+)\z/)) && questions.any?
          questions.last[:options] << { text: m[2], correct: m[1].downcase == "x" }
        elsif line.start_with?(">") && questions.any?
          questions.last[:explain] << line[1..].strip
        end
      end
      questions.select { |q| q[:options].any? }
    end

    # Inline Markdown (code, bold, links) without the wrapping <p>.
    def inline(text)
      Kramdown::Document.new(text.to_s, input: "GFM").to_html.strip.sub(%r{\A<p>(.*)</p>\z}m, '\1')
    end

    def h(text)
      CGI.escapeHTML(text.to_s)
    end
  end
end

Liquid::Template.register_tag("quiz", KevsRobots::QuizBlock)
