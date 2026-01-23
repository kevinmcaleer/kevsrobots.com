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

        // Human/bot visits breakdown
        const humanVisitsEl = document.getElementById('humanVisits');
        if (humanVisitsEl && data.totals.human_visits_last_month !== undefined) {
            humanVisitsEl.textContent = data.totals.human_visits_last_month.toLocaleString();
        }
        const botVisitsEl = document.getElementById('botVisits');
        if (botVisitsEl && data.totals.bot_visits_last_month !== undefined) {
            botVisitsEl.textContent = data.totals.bot_visits_last_month.toLocaleString();
        }

        const totalSearchesEl = document.getElementById('totalSearches');
        if (totalSearchesEl) totalSearchesEl.textContent = data.totals.searches_last_month.toLocaleString();

        const totalUsersEl = document.getElementById('totalUsers');
        if (totalUsersEl) totalUsersEl.textContent = data.user_stats.total_users.toLocaleString();

        // Human users breakdown
        const humanUsersEl = document.getElementById('humanUsers');
        if (humanUsersEl && data.user_stats.human_total_users !== undefined) {
            humanUsersEl.textContent = data.user_stats.human_total_users.toLocaleString();
        }

        const returningUsersEl = document.getElementById('returningUsers');
        if (returningUsersEl) returningUsersEl.textContent = data.user_stats.returning_users.toLocaleString();

        // Human returning users breakdown
        const humanReturningUsersEl = document.getElementById('humanReturningUsers');
        if (humanReturningUsersEl && data.user_stats.human_returning_users !== undefined) {
            humanReturningUsersEl.textContent = data.user_stats.human_returning_users.toLocaleString();
        }

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

        // =====================================================================
        // VIRAL DETECTION REPORTS
        // =====================================================================

        // Traffic Anomalies
        if (data.traffic_anomalies && data.traffic_anomalies.length > 0) {
            document.getElementById('anomalyAlert').style.display = 'block';
            renderTrafficAnomalies('trafficAnomalies', data.traffic_anomalies);
        }

        // Geographic Shifts
        if (data.geographic_shifts && data.geographic_shifts.length > 0) {
            document.getElementById('geoShiftAlert').style.display = 'block';
            renderGeographicShifts('geographicShifts', data.geographic_shifts);
        }

        // Trending Pages
        if (data.trending_pages) {
            renderTrendingPages('trendingPages', data.trending_pages);
        }

        // New vs Returning Visitors Chart
        if (data.new_visitor_surge && data.new_visitor_surge.length > 0) {
            renderNewVisitorChart('newVisitorChart', data.new_visitor_surge);
        }

        // Bot vs Human Traffic Chart
        if (data.bot_human_traffic && data.bot_human_traffic.length > 0) {
            renderBotHumanChart('botHumanChart', data.bot_human_traffic);
        }

        // Hourly Heatmap
        if (data.hourly_heatmap && data.hourly_heatmap.length > 0) {
            renderHourlyHeatmap('hourlyHeatmap', data.hourly_heatmap);
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

// =====================================================================
// VIRAL DETECTION RENDER FUNCTIONS
// =====================================================================

// Render traffic anomalies list
function renderTrafficAnomalies(elementId, anomalies) {
    const list = document.getElementById(elementId);
    list.innerHTML = anomalies.map((item, index) => {
        const url = new URL(item.url);
        const displayUrl = url.pathname;

        let badge = '';
        if (item.is_new) {
            badge = '<span class="anomaly-new">NEW PAGE</span>';
        } else if (item.pct_increase) {
            badge = `<span class="anomaly-spike">+${item.pct_increase}%</span>`;
        }

        return `
            <li class="query-item">
                <span>
                    <strong>#${index + 1}</strong>
                    <a href="${item.url}" target="_blank">${displayUrl}</a>
                    ${badge}
                    <span class="stats-badge visitors">${item.unique_visitors} unique</span>
                </span>
                <span class="stat-value" style="font-size: 20px;">${item.recent_visits.toLocaleString()}</span>
            </li>
        `;
    }).join('');
}

// Render geographic shifts list
function renderGeographicShifts(elementId, shifts) {
    const list = document.getElementById(elementId);
    list.innerHTML = shifts.map((item, index) => {
        let badge = '';
        if (item.pct_increase === 9999) {
            badge = '<span class="geo-surge">NEW</span>';
        } else {
            badge = `<span class="geo-surge">+${item.pct_increase}%</span>`;
        }

        return `
            <li class="query-item">
                <span>
                    <strong>#${index + 1}</strong>
                    ${item.country}
                    ${badge}
                    <small style="color: #6b7280;">(baseline: ${item.baseline_daily}/day)</small>
                </span>
                <span class="stat-value" style="font-size: 20px;">${item.recent_visits.toLocaleString()}</span>
            </li>
        `;
    }).join('');
}

// Render trending pages list
function renderTrendingPages(elementId, pages) {
    const list = document.getElementById(elementId);
    list.innerHTML = pages.slice(0, 10).map((item) => {
        const url = new URL(item.url);
        const displayUrl = url.pathname;

        let indicator = '';
        if (item.is_new) {
            indicator = '<span class="rank-new">NEW</span>';
        } else if (item.rank_change > 0) {
            indicator = `<span class="rank-up">↑${item.rank_change}</span>`;
        } else if (item.rank_change < 0) {
            indicator = `<span class="rank-down">↓${Math.abs(item.rank_change)}</span>`;
        }

        return `
            <li class="query-item">
                <span>
                    <strong>#${item.rank}</strong>
                    <a href="${item.url}" target="_blank">${displayUrl}</a>
                    ${indicator}
                    <span class="stats-badge visitors">${item.unique_visitors} unique</span>
                </span>
                <span class="stat-value" style="font-size: 20px;">${item.visits.toLocaleString()}</span>
            </li>
        `;
    }).join('');
}

// Render new vs returning visitors stacked area chart
function renderNewVisitorChart(canvasId, data) {
    const canvas = document.getElementById(canvasId);
    if (!canvas) return;

    new Chart(canvas, {
        type: 'bar',
        data: {
            labels: data.map(d => d.date),
            datasets: [
                {
                    label: 'New Visitors',
                    data: data.map(d => d.new_visitors),
                    backgroundColor: '#10b981',
                    stack: 'visitors'
                },
                {
                    label: 'Returning Visitors',
                    data: data.map(d => d.returning_visitors),
                    backgroundColor: '#6366f1',
                    stack: 'visitors'
                }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            plugins: {
                legend: {
                    position: 'top'
                },
                tooltip: {
                    callbacks: {
                        afterBody: function(context) {
                            const dataIndex = context[0].dataIndex;
                            const newPct = data[dataIndex].new_pct;
                            return `New visitor ratio: ${newPct}%`;
                        }
                    }
                }
            },
            scales: {
                x: {
                    stacked: true,
                    ticks: {
                        maxTicksLimit: 10
                    }
                },
                y: {
                    stacked: true,
                    beginAtZero: true
                }
            }
        }
    });
}

// Render bot vs human traffic stacked bar chart
function renderBotHumanChart(canvasId, data) {
    const canvas = document.getElementById(canvasId);
    if (!canvas) return;

    new Chart(canvas, {
        type: 'bar',
        data: {
            labels: data.map(d => d.date),
            datasets: [
                {
                    label: 'Human Traffic',
                    data: data.map(d => d.human),
                    backgroundColor: '#2563eb',
                    stack: 'traffic'
                },
                {
                    label: 'Google Bot',
                    data: data.map(d => d.google_bot),
                    backgroundColor: '#f59e0b',
                    stack: 'traffic'
                },
                {
                    label: 'Other Bots',
                    data: data.map(d => d.other_bots),
                    backgroundColor: '#9ca3af',
                    stack: 'traffic'
                }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            plugins: {
                legend: {
                    position: 'top'
                },
                tooltip: {
                    callbacks: {
                        afterBody: function(context) {
                            const dataIndex = context[0].dataIndex;
                            const humanPct = data[dataIndex].human_pct;
                            return `Human traffic: ${humanPct}%`;
                        }
                    }
                }
            },
            scales: {
                x: {
                    stacked: true,
                    ticks: {
                        maxTicksLimit: 10
                    }
                },
                y: {
                    stacked: true,
                    beginAtZero: true
                }
            }
        }
    });
}

// Render hourly heatmap as a matrix chart
function renderHourlyHeatmap(canvasId, data) {
    const canvas = document.getElementById(canvasId);
    if (!canvas) return;

    // Group data by date and hour
    const dates = [...new Set(data.map(d => d.date))].sort();
    const hours = Array.from({length: 24}, (_, i) => i);

    // Create datasets for each date (row)
    const datasets = dates.map((date, dateIndex) => {
        const dayData = data.filter(d => d.date === date);
        const hourlyVisits = hours.map(hour => {
            const found = dayData.find(d => d.hour === hour);
            return found ? found.visits : 0;
        });

        // Calculate color intensity based on max value
        const maxVisits = Math.max(...data.map(d => d.visits));

        return {
            label: date,
            data: hourlyVisits,
            backgroundColor: hourlyVisits.map(v => {
                const intensity = v / maxVisits;
                return `rgba(37, 99, 235, ${0.1 + intensity * 0.9})`;
            }),
            borderWidth: 1,
            borderColor: '#e5e7eb'
        };
    });

    // Use a simple bar chart with hours on x-axis
    // Average across all days for simplicity
    const avgByHour = hours.map(hour => {
        const hourData = data.filter(d => d.hour === hour);
        return hourData.length > 0
            ? Math.round(hourData.reduce((sum, d) => sum + d.visits, 0) / hourData.length)
            : 0;
    });

    new Chart(canvas, {
        type: 'bar',
        data: {
            labels: hours.map(h => `${h}:00`),
            datasets: [{
                label: 'Avg Visits/Hour',
                data: avgByHour,
                backgroundColor: avgByHour.map(v => {
                    const maxV = Math.max(...avgByHour);
                    const intensity = v / maxV;
                    if (intensity > 0.8) return '#1e40af';
                    if (intensity > 0.6) return '#2563eb';
                    if (intensity > 0.4) return '#3b82f6';
                    if (intensity > 0.2) return '#60a5fa';
                    return '#93c5fd';
                }),
                borderWidth: 0
            }]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            plugins: {
                legend: {
                    display: false
                },
                tooltip: {
                    callbacks: {
                        title: function(context) {
                            return `Hour: ${context[0].label}`;
                        },
                        label: function(context) {
                            return `Avg visits: ${context.parsed.y.toLocaleString()}`;
                        }
                    }
                }
            },
            scales: {
                x: {
                    title: {
                        display: true,
                        text: 'Hour (UTC)'
                    }
                },
                y: {
                    beginAtZero: true,
                    title: {
                        display: true,
                        text: 'Avg Visits'
                    }
                }
            }
        }
    });
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
