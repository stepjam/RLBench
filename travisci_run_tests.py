from unittest import TestSuite, defaultTestLoader
from HtmlTestRunner import HTMLTestRunner
import sys
import os

TEST_TYPES = ['unit', 'demos']


if __name__ == '__main__':

    test_folder = 'tests'

    assert len(sys.argv) == 4

    test_type = sys.argv[1]
    branch = sys.argv[2]
    build_num = sys.argv[3]

    assert test_type in TEST_TYPES

    test_suite = TestSuite()
    all_test_cases = defaultTestLoader.discover(
        os.path.join(test_folder, test_type), 'test*.py')
    # Loop the found test cases and add them into test suite.
    for test_case in all_test_cases:
        test_suite.addTests(test_case)
    runner = HTMLTestRunner(
        output='reports/%s' % str(build_num),
        report_name=test_type,
        add_timestamp=False, combine_reports=True)
    result = runner.run(test_suite)

    # Will be 0 if tests ran OK
    sys.exit(1 if len(result.failures) + len(result.errors) > 0 else 0)
