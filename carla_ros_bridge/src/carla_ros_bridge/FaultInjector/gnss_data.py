class GNSSData:
    """
    A shared class to store and provide the latest GNSS data.
    """
    _current_location = None

    @classmethod
    def update_location(cls, location):
        """
        Update the current GNSS location.

        :param location: A dictionary with keys 'latitude', 'longitude', 'altitude'.
        """
        cls._current_location = location

    @classmethod
    def get_location(cls):
        """
        Get the current GNSS location.

        :return: A dictionary with keys 'latitude', 'longitude', 'altitude', or None if not set.
        """
        return cls._current_location