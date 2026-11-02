import unittest
from create_diagnostic_rviz import configure


class DiagnosticProfileTest(unittest.TestCase):
    def test_separates_sources_and_corrects_topics(self):
        names = ['/wheelchair/local_costmap/costmap',
                 '/wheelchair/local_costmap/published_footprint',
                 '/mid360_201/points_filtered', '/mid360_202/points_filtered',
                 '/odin1/cloud_render']
        source = {'Visualization Manager': {'Global Options': {'Fixed Frame': 'odom'}},
                  'Displays': [{'Topic': {'Value': n}, 'Enabled': True} for n in names]}
        result = configure(source)
        displays = result['Displays']
        self.assertEqual(displays[0]['Topic']['Value'], '/local_costmap/costmap')
        self.assertEqual(displays[1]['Topic']['Value'], '/local_costmap/published_footprint')
        self.assertEqual(displays[2]['Decay Time'], .5)
        self.assertTrue(displays[3]['Enabled'])
        self.assertFalse(displays[4]['Enabled'])
        self.assertEqual(result['Visualization Manager']['Global Options']['Fixed Frame'], 'base_link')
        self.assertEqual(source['Displays'][0]['Topic']['Value'], names[0])

    def test_missing_layer_fails(self):
        with self.assertRaises(ValueError):
            configure({'Visualization Manager': {'Global Options': {}}, 'Displays': []})


if __name__ == '__main__':
    unittest.main()
