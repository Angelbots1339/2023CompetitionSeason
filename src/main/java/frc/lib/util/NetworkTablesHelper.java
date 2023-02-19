package frc.lib.util;

import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTablesHelper {

    /**
     * Gets a boolean from given table/entry
     * @param tableName
     * @param entryName
     * @return Networktables entry, if exists. Defaults to FALSE otherwise
     */
    public static boolean getBoolean(String tableName, String entryName) {
        return getBoolean(tableName, entryName, false);
    }

    /**
     * Gets a boolean from given table/entry
     * @param tableName
     * @param entryName
     * @param defaultEntry Boolean to return if entry doesn't exist
     * @return Networktables entry, if exists.
     */
    public static boolean getBoolean(String tableName, String entryName, boolean defaultEntry) {
        return NetworkTableInstance
            .getDefault()
            .getTable(tableName)
            .getEntry(entryName)
            .getBoolean(defaultEntry);
    }

    /**
     * Gets a boolean from given table/subtable/entry
     * @param tableName
     * @param subTableName
     * @param entryName
     * @return Networktables entry, if exists. Defaults to FALSE otherwise
     */
    public static boolean getBoolean(String tableName, String subTableName, String entryName) {
        return getBoolean(tableName, subTableName, entryName, false);
    }

    /**
     * Gets a boolean from given table/subtable/entry
     * @param tableName
     * @param subTableName
     * @param entryName
     * @param defaultEntry
     * @return Networktables entry, if exists.
     */
    public static boolean getBoolean(String tableName, String subTableName, String entryName, boolean defaultEntry) {
        return NetworkTableInstance
            .getDefault()
            .getTable(tableName)
            .getSubTable(subTableName)
            .getEntry(entryName)
            .getBoolean(defaultEntry);
    }
    
    /**
     * Gets a double from given table/entry
     * @param tableName
     * @param entryName
     * @return Networktables entry, if exists. Defaults to 0 otherwise
     */
    public static double getDouble(String tableName, String entryName) {
        return getDouble(tableName, entryName, 0);
    }

    /**
     * Gets a double from given table/entry
     * @param tableName
     * @param entryName
     * @param defaultEntry Double to return if entry doesn't exist
     * @return Networktables entry, if exists.
     */
    public static double getDouble(String tableName, String entryName, double defaultEntry) {
        return NetworkTableInstance
            .getDefault()
            .getTable(tableName)
            .getEntry(entryName)
            .getDouble(defaultEntry);
    }

    /**
     * Gets a double from given table/subtable/entry
     * @param tableName
     * @param subTableName
     * @param entryName
     * @return Networktables entry, if exists. Defaults to 0 otherwise
     */
    public static double getDouble(String tableName, String subTableName, String entryName) {
        return getDouble(tableName, subTableName, entryName, 0);
    }

    /**
     * Gets a double from given table/subtable/entry
     * @param tableName
     * @param subTableName
     * @param entryName
     * @param defaultEntry
     * @return Networktables entry, if exists.
     */
    public static double getDouble(String tableName, String subTableName, String entryName, double defaultEntry) {
        return NetworkTableInstance
            .getDefault()
            .getTable(tableName)
            .getSubTable(subTableName)
            .getEntry(entryName)
            .getDouble(defaultEntry);
    }

    /**
     * Sets a boolean from given table/entry
     * @param tableName
     * @param entryName
     * @param entry
     * @return False if the entry exists with a different type
     */
    public static boolean setBoolean(String tableName, String entryName, boolean entry) {
        return NetworkTableInstance
            .getDefault()
            .getTable(tableName)
            .getEntry(entryName)
            .setBoolean(entry);
    }

    /**
     * Sets a boolean from given table/subtable/entry
     * @param tableName
     * @param subTableName
     * @param entryName
     * @param entry
     * @return False if the entry exists with a different type
     */
    public static boolean setBoolean(String tableName, String subTableName, String entryName, boolean entry) {
        return NetworkTableInstance
            .getDefault()
            .getTable(tableName)
            .getSubTable(subTableName)
            .getEntry(entryName)
            .setBoolean(entry);
    }

    /**
     * Sets a double from given table/entry
     * @param tableName
     * @param entryName
     * @param entry
     * @return False if the entry exists with a different type
     */
    public static boolean setDouble(String tableName, String entryName, double entry) {
        return NetworkTableInstance
            .getDefault()
            .getTable(tableName)
            .getEntry(entryName)
            .setDouble(entry);
    }

    /**
     * Sets a double from given table/subtable/entry
     * @param tableName
     * @param subTableName
     * @param entryName
     * @param entry
     * @return False if the entry exists with a different type
     */
    public static boolean setDouble(String tableName, String subTableName, String entryName, double entry) {
        return NetworkTableInstance
            .getDefault()
            .getTable(tableName)
            .getSubTable(subTableName)
            .getEntry(entryName)
            .setDouble(entry);
    }
}
