// Configuration
// Use direct IP for local development to bypass CORS temporarily
const API_URL = window.location.hostname.includes('local')
    ? 'http://192.168.2.4:8008/api/dashboard'
    : 'https://stats.kevsrobots.com/api/dashboard';

console.log('Stats Dashboard: Loaded with API_URL:', API_URL);

// Fetch and render dashboard data
async function loadDashboard() {
    console.log('Stats Dashboard: loadDashboard() called');
    try {
        console.log('Stats Dashboard: Fetching from', API_URL);
        const response = await fetch(API_URL);
        console.log('Stats Dashboard: Response status:', response.status);
        const data = await response.json();
        console.log('Stats Dashboard: Data received:', data);

        // Update summary stats
        console.log('Stats Dashboard: Updating summary stats');
        const totalVisitsEl = document.getElementById('totalVisits');
        console.log('Stats Dashboard: totalVisits element:', totalVisitsEl);
        if (totalVisitsEl) totalVisitsEl.textContent = data.totals.visits_last_month.toLocaleString();

        const totalSearchesEl = document.getElementById('totalSearches');
        if (totalSearchesEl) totalSearchesEl.textContent = data.totals.searches_last_month.toLocaleString();

        const totalUsersEl = document.getElementById('totalUsers');
        if (totalUsersEl) totalUsersEl.textContent = data.user_stats.total_users.toLocaleString();

        const returningUsersEl = document.getElementById('returningUsers');
        if (returningUsersEl) returningUsersEl.textContent = data.user_stats.returning_users.toLocaleString();

        // Visits per day chart (bar chart)
        renderBarChart(
            'visitsPerDayChart',
            'Visits per Day',
            data.visits_per_day.map(d => d.date),
            data.visits_per_day.map(d => d.count)
        );

        // Searches per day chart
        renderLineChart(
            'searchesPerDayChart',
            'Searches per Day',
            data.searches_per_day.map(d => d.date),
            data.searches_per_day.map(d => d.count)
        );

        // Top searches list
        renderTopSearches('topSearchesWeek', data.top_searches_week);

        // Trending searches with movement indicators
        renderTrendingSearches('trendingSearches', data.search_trends);

        // Device breakdown pie chart
        renderPieChart(
            'deviceChart',
            'Devices',
            Object.keys(data.device_breakdown),
            Object.values(data.device_breakdown)
        );

        // OS breakdown pie chart
        renderPieChart(
            'osChart',
            'Operating Systems',
            Object.keys(data.os_breakdown),
            Object.values(data.os_breakdown)
        );

        // Page type breakdown pie chart
        renderPieChart(
            'pageTypeChart',
            'Content Types',
            Object.keys(data.page_type_breakdown),
            Object.values(data.page_type_breakdown)
        );

        // Popular pages list
        renderPopularPages('popularPages', data.popular_pages);

        // Country breakdown horizontal bar chart
        if (data.country_breakdown && data.country_breakdown.length > 0) {
            renderCountryChart(
                'countryChart',
                data.country_breakdown.map(c => c.country),
                data.country_breakdown.map(c => c.visits)
            );
        }

        // Update timestamp
        const lastUpdate = new Date(data.last_updated);
        document.getElementById('lastUpdated').textContent =
            `Last updated: ${lastUpdate.toLocaleString()}`;

    } catch (error) {
        console.error('Stats Dashboard: Error loading dashboard:', error);
        console.error('Stats Dashboard: Error stack:', error.stack);
        const lastUpdatedEl = document.getElementById('lastUpdated');
        if (lastUpdatedEl) {
            lastUpdatedEl.textContent = 'Error loading data. Please try again later.';
        }
    }
}

// Render line chart
function renderLineChart(canvasId, label, labels, data) {
    new Chart(document.getElementById(canvasId), {
        type: 'line',
        data: {
            labels: labels,
            datasets: [{
                label: label,
                data: data,
                borderColor: '#2563eb',
                backgroundColor: 'rgba(37, 99, 235, 0.1)',
                tension: 0.4,
                fill: true
            }]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            plugins: {
                legend: {
                    display: false
                }
            },
            scales: {
                x: {
                    display: true,
                    ticks: {
                        maxTicksLimit: 12
                    }
                },
                y: {
                    beginAtZero: true
                }
            }
        }
    });
}

// Render bar chart
function renderBarChart(canvasId, label, labels, data) {
    new Chart(document.getElementById(canvasId), {
        type: 'bar',
        data: {
            labels: labels,
            datasets: [{
                label: label,
                data: data,
                backgroundColor: '#2563eb',
                borderColor: '#1e40af',
                borderWidth: 1
            }]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            plugins: {
                legend: {
                    display: false
                }
            },
            scales: {
                x: {
                    display: true,
                    ticks: {
                        maxTicksLimit: 12
                    }
                },
                y: {
                    beginAtZero: true
                }
            }
        }
    });
}

// Render pie chart
function renderPieChart(canvasId, label, labels, data) {
    new Chart(document.getElementById(canvasId), {
        type: 'pie',
        data: {
            labels: labels,
            datasets: [{
                label: label,
                data: data,
                backgroundColor: [
                    '#2563eb',
                    '#10b981',
                    '#f59e0b',
                    '#ef4444',
                    '#8b5cf6',
                    '#ec4899'
                ]
            }]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            plugins: {
                legend: {
                    position: 'right'
                }
            }
        }
    });
}

// Render country breakdown horizontal bar chart
function renderCountryChart(canvasId, countries, visits) {
    const canvas = document.getElementById(canvasId);
    if (!canvas) return;

    new Chart(canvas, {
        type: 'bar',
        data: {
            labels: countries,
            datasets: [{
                label: 'Visits',
                data: visits,
                backgroundColor: '#2563eb',
                borderColor: '#1e40af',
                borderWidth: 1
            }]
        },
        options: {
            indexAxis: 'y',  // Horizontal bars
            responsive: true,
            maintainAspectRatio: false,
            plugins: {
                legend: {
                    display: false
                },
                tooltip: {
                    callbacks: {
                        label: function(context) {
                            return `${context.parsed.x.toLocaleString()} visits`;
                        }
                    }
                }
            },
            scales: {
                x: {
                    beginAtZero: true,
                    ticks: {
                        callback: function(value) {
                            return value.toLocaleString();
                        }
                    }
                }
            }
        }
    });
}

// Render top searches list
function renderTopSearches(elementId, searches) {
    const list = document.getElementById(elementId);
    list.innerHTML = searches.map((item, index) => `
        <li class="query-item">
            <span><strong>#${index + 1}</strong> <a href="/search-results.html?query=${encodeURIComponent(item.query)}&page=1">${item.query}</a></span>
            <span class="stat-value" style="font-size: 20px;">${item.count}</span>
        </li>
    `).join('');
}

// Render trending searches with indicators
function renderTrendingSearches(elementId, trends) {
    const list = document.getElementById(elementId);
    list.innerHTML = trends.map((item, index) => {
        let indicator = '';
        if (item.is_new) {
            indicator = '<span class="trend-new">NEW</span>';
        } else if (item.rank_change > 0) {
            indicator = `<span class="trend-up">↑${item.rank_change}</span>`;
        } else if (item.rank_change < 0) {
            indicator = `<span class="trend-down">↓${Math.abs(item.rank_change)}</span>`;
        }

        return `
            <li class="query-item">
                <span><strong>#${item.rank}</strong> <a href="/search-results.html?query=${encodeURIComponent(item.query)}&page=1">${item.query}</a> ${indicator}</span>
                <span class="stat-value" style="font-size: 20px;">${item.count}</span>
            </li>
        `;
    }).join('');
}

// Render popular pages list
function renderPopularPages(elementId, pages) {
    const list = document.getElementById(elementId);
    list.innerHTML = pages.slice(0, 10).map((page, index) => {
        const url = new URL(page.url);
        const displayUrl = url.pathname;

        return `
            <li class="query-item">
                <span>
                    <strong>#${index + 1}</strong>
                    <a href="${page.url}" target="_blank">${displayUrl}</a>
                    <small style="color: #6b7280;"> (${page.page_type})</small>
                </span>
                <span class="stat-value" style="font-size: 20px;">${page.visits.toLocaleString()}</span>
            </li>
        `;
    }).join('');
}

// Load dashboard on page load
console.log('Stats Dashboard: Registering load handler, document.readyState:', document.readyState);
if (document.readyState === 'loading') {
    console.log('Stats Dashboard: Document still loading, adding DOMContentLoaded listener');
    document.addEventListener('DOMContentLoaded', () => {
        console.log('Stats Dashboard: DOMContentLoaded fired');
        loadDashboard();
    });
} else {
    console.log('Stats Dashboard: Document already loaded, calling loadDashboard immediately');
    loadDashboard();
}

// Refresh every 5 minutes
setInterval(loadDashboard, 5 * 60 * 1000);
