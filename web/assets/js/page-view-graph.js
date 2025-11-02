/**
 * Mini Page View Graph Component
 *
 * Displays a minimal, clickable graph showing page view trends.
 * Click to cycle through weekly, monthly, and annual views.
 *
 * Usage:
 *   <div class="page-view-graph" data-url="/blog/my-post.html"></div>
 *   <script src="/assets/js/page-view-graph.js"></script>
 */

(function() {
  'use strict';

  const CHATTER_API = 'https://chatter.kevsrobots.com/analytics';
  const PERIODS = ['weekly', 'monthly', 'annual'];

  // Initialize all graphs on page load
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }

  function init() {
    const graphs = document.querySelectorAll('.page-view-graph');
    graphs.forEach(graph => {
      if (!graph.dataset.initialized) {
        initGraph(graph);
        graph.dataset.initialized = 'true';
      }
    });
  }

  function initGraph(container) {
    const url = container.dataset.url;
    if (!url) {
      console.error('[PageViewGraph] No data-url attribute found');
      return;
    }

    // Initial state
    let currentPeriodIndex = 0;
    let currentData = null;

    // Create graph structure
    container.innerHTML = `
      <div class="page-view-graph-inner" role="button" tabindex="0"
           aria-label="Page view graph, click to cycle views"
           title="Click to cycle: Week → Month → Year">
        <svg class="graph-svg" width="200" height="60"
             style="display: block; cursor: pointer;"></svg>
        <div class="graph-loading" style="display: none; padding: 20px; text-align: center; color: #666;">
          Loading...
        </div>
        <div class="graph-error" style="display: none; padding: 10px; text-align: center; color: #999; font-size: 0.85em;">
          No data
        </div>
      </div>
    `;

    const svg = container.querySelector('.graph-svg');
    const loading = container.querySelector('.graph-loading');
    const error = container.querySelector('.graph-error');
    const inner = container.querySelector('.page-view-graph-inner');

    // Load initial data
    loadAndRender();

    // Click to cycle periods
    inner.addEventListener('click', cyclePeriod);
    inner.addEventListener('keydown', (e) => {
      if (e.key === 'Enter' || e.key === ' ') {
        e.preventDefault();
        cyclePeriod();
      }
    });

    async function loadAndRender() {
      const period = PERIODS[currentPeriodIndex];

      // Show loading
      loading.style.display = 'block';
      svg.style.display = 'none';
      error.style.display = 'none';

      try {
        const response = await fetch(
          `${CHATTER_API}/page-views/${encodeURIComponent(url)}/timeline?period=${period}`,
          { credentials: 'include' }
        );

        if (!response.ok) {
          throw new Error(`HTTP ${response.status}`);
        }

        currentData = await response.json();

        // Hide loading, show graph
        loading.style.display = 'none';

        if (currentData.total_views === 0) {
          error.style.display = 'block';
        } else {
          svg.style.display = 'block';
          renderGraph(svg, currentData);
        }

      } catch (err) {
        console.error('[PageViewGraph] Error loading data:', err);
        loading.style.display = 'none';
        error.style.display = 'block';
      }
    }

    function cyclePeriod() {
      currentPeriodIndex = (currentPeriodIndex + 1) % PERIODS.length;
      loadAndRender();
    }

    function renderGraph(svg, data) {
      // Clear existing content
      svg.innerHTML = '';

      const { period, data: dataPoints } = data;
      const counts = dataPoints.map(d => d.count);
      const labels = dataPoints.map(d => d.label);

      const maxCount = Math.max(...counts, 1); // Avoid division by zero
      const width = 200;
      const height = 60;
      const padding = { top: 5, right: 5, bottom: 15, left: 5 };
      const graphWidth = width - padding.left - padding.right;
      const graphHeight = height - padding.top - padding.bottom;

      // Create SVG group for graph content
      const g = document.createElementNS('http://www.w3.org/2000/svg', 'g');
      g.setAttribute('transform', `translate(${padding.left},${padding.top})`);
      svg.appendChild(g);

      // Render based on period type
      if (period === 'weekly' || period === 'annual') {
        // Bar chart
        renderBarChart(g, counts, labels, graphWidth, graphHeight, maxCount);
      } else {
        // Line chart for monthly
        renderLineChart(g, counts, labels, graphWidth, graphHeight, maxCount);
      }
    }

    function renderBarChart(g, counts, labels, width, height, maxCount) {
      const barCount = counts.length;
      const barWidth = (width / barCount) * 0.7; // 70% width, 30% gap
      const gap = (width / barCount) * 0.3;

      counts.forEach((count, i) => {
        const barHeight = (count / maxCount) * height;
        const x = (i * width / barCount) + (gap / 2);
        const y = height - barHeight;

        // Bar
        const rect = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
        rect.setAttribute('x', x);
        rect.setAttribute('y', y);
        rect.setAttribute('width', barWidth);
        rect.setAttribute('height', barHeight);
        rect.setAttribute('fill', '#000');
        rect.setAttribute('opacity', '0.8');
        g.appendChild(rect);

        // Label
        const text = document.createElementNS('http://www.w3.org/2000/svg', 'text');
        text.setAttribute('x', x + barWidth / 2);
        text.setAttribute('y', height + 12);
        text.setAttribute('text-anchor', 'middle');
        text.setAttribute('font-size', '9');
        text.setAttribute('font-family', 'system-ui, sans-serif');
        text.setAttribute('fill', '#000');
        text.textContent = labels[i];
        g.appendChild(text);
      });
    }

    function renderLineChart(g, counts, labels, width, height, maxCount) {
      const pointCount = counts.length;
      const xStep = width / (pointCount - 1 || 1);

      // Build path
      let pathData = '';
      const points = [];

      counts.forEach((count, i) => {
        const x = i * xStep;
        const y = height - (count / maxCount) * height;
        points.push({ x, y });

        if (i === 0) {
          pathData += `M ${x} ${y}`;
        } else {
          pathData += ` L ${x} ${y}`;
        }
      });

      // Line
      const path = document.createElementNS('http://www.w3.org/2000/svg', 'path');
      path.setAttribute('d', pathData);
      path.setAttribute('fill', 'none');
      path.setAttribute('stroke', '#000');
      path.setAttribute('stroke-width', '2');
      path.setAttribute('opacity', '0.8');
      g.appendChild(path);

      // Points
      points.forEach(point => {
        const circle = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
        circle.setAttribute('cx', point.x);
        circle.setAttribute('cy', point.y);
        circle.setAttribute('r', '2');
        circle.setAttribute('fill', '#000');
        g.appendChild(circle);
      });

      // Labels (show every 3rd for monthly to avoid crowding)
      labels.forEach((label, i) => {
        if (i % 3 === 0 || i === labels.length - 1) {
          const text = document.createElementNS('http://www.w3.org/2000/svg', 'text');
          text.setAttribute('x', i * xStep);
          text.setAttribute('y', height + 12);
          text.setAttribute('text-anchor', 'middle');
          text.setAttribute('font-size', '9');
          text.setAttribute('font-family', 'system-ui, sans-serif');
          text.setAttribute('fill', '#000');
          text.textContent = label;
          g.appendChild(text);
        }
      });
    }
  }
})();
