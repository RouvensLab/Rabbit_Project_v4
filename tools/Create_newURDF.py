import xml.etree.ElementTree as ET

class URDFModifier:
    def __init__(self, urdf_file):
        self.urdf_file = urdf_file
        self.masses = {
            "Kopf": 0.121,
            "Vorderläufe": 0.024,
            "Oberkörper": 0.171,
            "Mittelkörper": 0.291,
            "Unterkörper": 0.311,
            "Hinterläufe": 0.082
        }
        self.body_parts = {
            "Kopf": {4: 0.8, 3: 0.2},
            "Vorderläufe": {6: 0.5, 5: 0.5},
            "Oberkörper": {1: 0.25, 2: 0.25, 7: 0.25, 8: 0.25},
            "Mittelkörper": {0: 0.25, 9: 0.25, 22: 0.25, 23: 0.25/3, 10: 0.25/3, 11: 0.25/3},
            "Unterkörper": {19: 0.2, 21: 0.2, 18: 0.2, 20: 0.2, 11: 0.2},
            "Hinterläufe": {12: 0.2, 15: 0.2, 13: 0.2, 16: 0.2, 14: 0.1, 17: 0.1}
        }

    def find_mass(self, index):
        """return
        mass: float
        
        """
        for key, value in self.body_parts.items():
            if index in value:
                return self.masses[key] * value[index]
        return None

    def modify_masses(self):
        tree = ET.parse(self.urdf_file)
        root = tree.getroot()

        for index, link in enumerate(root.iter('link')):
            link_name = link.attrib['name']
            new_mass = self.find_mass(index)
            if new_mass is not None:
                link.find('inertial/mass').set('value', str(new_mass))

        tree.write(self.urdf_file)

if __name__ == "__main__":
    urdf_modifier = URDFModifier(r'URDFs\URDF_description\urdf\URDF_realRabbit.xacro')
    urdf_modifier.modify_masses()