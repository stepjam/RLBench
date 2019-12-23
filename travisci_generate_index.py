import sys
import os
import re
import shutil

TEST_TYPES = ['unit', 'demos']
HISTORY_TO_KEEP = 30


if __name__ == '__main__':

    assert len(sys.argv) == 5

    test_type = sys.argv[1]
    branch = sys.argv[2]
    build_num = sys.argv[3]
    exit_code = sys.argv[4]

    fail = int(exit_code) != 0

    assert test_type in TEST_TYPES

    uls = []
    if os.path.exists('index.html'):
        with open('index.html', 'r') as f:
            lines = f.read()
        uls = [out[0] for out in re.findall("(<ul>((.|\n)*?)</ul>)", lines)]
        if len(uls) > HISTORY_TO_KEEP - 1:
            uls = uls[:HISTORY_TO_KEEP - 1]

    reports = sorted(os.listdir('reports'))

    if len(reports) > HISTORY_TO_KEEP:
        for delete_folder in reports[:-HISTORY_TO_KEEP]:
            shutil.rmtree(os.path.join('reports', delete_folder))

    color = "style='color:Tomato;'" if fail else "style='color:MediumSeaGreen;'"
    new_entry = ("<ul><a %s href='reports/%s/%s.html'>Build %s. Branch: %s. "
                 "Test: %s. Status: %s</a></ul>\n" %
                 (color, build_num, test_type, build_num, branch, test_type,
                  'Fail!' if fail else 'Pass!'))

    html_top = """
    <!DOCTYPE html>
    <html>
    <body>
    <h1>RLBench Test Page</h1>
    """
    html_bottom = """
    </body>
    </html>
    """

    new_html = html_top + new_entry + '\n'.join(uls) + html_bottom
    with open('index.html', 'w') as f:
        f.write(new_html)

    # Will be 0 if tests ran OK
    sys.exit(fail)
