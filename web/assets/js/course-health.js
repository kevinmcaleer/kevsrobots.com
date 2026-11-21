// Course health (#637): fills the Helpful column from Nibsy's lesson votes.
(function () {
    const NIBSY_API = 'https://nibsy.kevsrobots.com';
    const cells = document.querySelectorAll('tr[data-course-key] [data-helpful]');
    fetch(NIBSY_API + '/api/learn/helpful')
        .then(res => (res.ok ? res.json() : null))
        .then(body => {
            const courses = (body && body.courses) || {};
            cells.forEach(cell => {
                const votes = courses[cell.closest('tr').dataset.courseKey];
                const total = votes ? votes.up + votes.down : 0;
                if (!total) { cell.textContent = 'no votes'; return; }
                cell.textContent = Math.round((votes.up / total) * 100) + '% ';
                const n = document.createElement('small');
                n.textContent = `(${total})`;
                cell.appendChild(n);
            });
        })
        .catch(() => cells.forEach(cell => { cell.textContent = 'n/a'; }));
})();
