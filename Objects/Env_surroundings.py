import pybullet as p
import pybullet_data
import numpy as np


class Terrain:
    def __init__(self, terrain_type="uneven_terrain", platform_size=70):
        self.terrain_type = terrain_type
        self.platform_size = platform_size
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        if terrain_type == "uneven_terrain":
            # Create heightfield data: a grid of random heights to simulate uneven terrain
            num_rows = self.platform_size  # Number of rows in the heightfield grid
            num_columns = self.platform_size  # Number of columns in the heightfield grid
            height_scale = 0.1  # Scale the height values to control the roughness of the terrain

            # Generate random heights for the terrain
            heightfield_data = np.random.uniform(-1, 1, size=(num_rows, num_columns))
            heightfield_data = heightfield_data * height_scale

            # Flatten the heightfield data into a 1D array, as required by PyBullet
            heightfield_data_flattened = heightfield_data.flatten()

            # Create heightfield collision shape
            # random meshScale, range from 0.05 to 0.5
            rand_meshScale = [0.1, 0.1, 0.1]  # np.random.uniform(0.05, 1, 3)

            terrain_shape = p.createCollisionShape(
                shapeType=p.GEOM_HEIGHTFIELD,
                meshScale=rand_meshScale,  # Scale of the heightfield
                heightfieldTextureScaling=(num_rows - 1) / 2,
                heightfieldData=heightfield_data_flattened,
                numHeightfieldRows=num_rows,
                numHeightfieldColumns=num_columns
            )

            # Create the terrain in the simulation
            self._id = p.createMultiBody(0, terrain_shape)

            # Optionally, you can set additional visual or physical properties for the terrain
            p.resetBasePositionAndOrientation(self._id, [0, 0, 0], [0, 0, 0, 1])

        elif self.terrain_type == "random_terrain":
            # Create heightfield data: a grid of random heights to simulate uneven terrain
            num_rows = self.platform_size  # Number of rows in the heightfield grid
            num_columns = self.platform_size  # Number of columns in the heightfield grid
            height_scale = 0.2  # Adjust this for terrain roughness (forest ground shouldn't be too extreme)

            # Generate random heights for the terrain (use smoother ranges for natural feel)
            heightfield_data = np.random.uniform(-0.5, 0.5, size=(num_rows, num_columns))
            heightfield_data = heightfield_data * height_scale

            # Flatten the heightfield data into a 1D array, as required by PyBullet
            heightfield_data_flattened = heightfield_data.flatten()

            # Create heightfield collision shape
            # Create heightfield collision shape with varied meshScale for more natural terrain
            meshScale = [
                np.random.uniform(0.08, 0.22),  # Scale for x-axis (small variation)
                np.random.uniform(0.08, 0.22),  # Scale for y-axis (small variation)
                np.random.uniform(0.1, 0.35)    # Larger variation for z-axis to simulate bumps and dips
            ]

            terrain_shape = p.createCollisionShape(
                shapeType=p.GEOM_HEIGHTFIELD,
                meshScale=meshScale,  # Scale of the heightfield
                heightfieldTextureScaling=(num_rows - 1) / 2,
                heightfieldData=heightfield_data_flattened,
                numHeightfieldRows=num_rows,
                numHeightfieldColumns=num_columns
            )

            # Create the terrain in the simulation
            self._id = p.createMultiBody(0, terrain_shape)

            # Optionally, set additional visual or physical properties for the terrain
            p.resetBasePositionAndOrientation(self._id, [0, 0, 0], [0, 0, 0, 1])

            # You could also apply textures or additional friction properties if needed
        else:
            self._id = p.loadURDF("plane.urdf")

        # Simulate a ground like a carpet
        lateralFriction = 0.5
        restitution = 0.5
        rollingFriction = 0.1
        # Set the ne dynamics
        p.changeDynamics(self._id, -1, lateralFriction=lateralFriction, restitution=restitution, rollingFriction=rollingFriction)

    def reset(self):
        # Reset the terrain if needed (e.g., re-generate the heightfield)
        p.removeBody(self._id)
        self.__init__(self.terrain_type, self.platform_size)


    @property
    def id(self):
        return self._id