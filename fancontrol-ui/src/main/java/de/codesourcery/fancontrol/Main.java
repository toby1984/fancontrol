package de.codesourcery.fancontrol;

import org.apache.commons.lang3.Validate;

import javax.swing.*;
import java.awt.*;
import java.io.*;
import java.lang.reflect.InvocationTargetException;
import java.util.List;

public class Main extends JFrame
{
    private final FanSpeedMappingEditor panel = new FanSpeedMappingEditor();

    private JComboBox<ThermalZone> zoneSelector = new JComboBox<>();

    public interface IConfigProvider {

        Config load();
        void save(Config config);
    }

    public static final class ClasspathConfigProvider implements IConfigProvider
    {
        private final String classpath;

        public ClasspathConfigProvider(String classpath)
        {
            Validate.notBlank( classpath, "classpath must not be null or blank");
            this.classpath = classpath;
        }

        @Override
        public Config load()
        {
            final InputStream in = Main.class.getResourceAsStream(classpath);
            try
            {
                return Config.fromJSON(in);
            }
            catch (IOException e)
            {
                throw new RuntimeException("Failed to load config from classpath "+classpath);
            }
        }

        @Override
        public void save(Config config) { /* nop */ }
    }

    public static final class FileConfigProvider implements IConfigProvider {

        private final File file;

        public FileConfigProvider(File file)
        {
            Validate.notNull(file, "file must not be null");
            this.file = file;
        }

        @Override
        public Config load()
        {
            try ( FileInputStream in = new FileInputStream(file ) )
            {
                return Config.fromJSON(in);
            }
            catch(Exception e)
            {
                throw new RuntimeException(e);
            }
        }

        @Override
        public void save(Config config)
        {
            try (FileWriter out = new FileWriter(file) ) {
                out.write( config.toJSON() );
            }
            catch(Exception e)
            {
                throw new RuntimeException(e);
            }
        }
    }

    public Main(IConfigProvider configProvider)
    {
        super("fancontrol UI");
        Validate.notNull(configProvider, "configProvider must not be null");

        final JPanel controls = new JPanel();
        controls.setLayout(new FlowLayout());

        final Config config = configProvider.load();
        final List<ThermalZone> zones = config.thermalZones;

        // Thermal zone combo box
        final ComboBoxModel<ThermalZone> comboModel = new DefaultComboBoxModel<>( zones.toArray(new ThermalZone[0] ));

        zoneSelector.setRenderer(new DefaultListCellRenderer() {
            @Override
            public Component getListCellRendererComponent(JList<?> list, Object value, int index, boolean isSelected, boolean cellHasFocus)
            {
                final Component result = super.getListCellRendererComponent(list, value, index, isSelected, cellHasFocus);
                if ( value instanceof ThermalZone ) {
                    setText( ((ThermalZone) value).name );
                }
                return result;
            }
        });
        zoneSelector.setModel(comboModel);
        zoneSelector.addActionListener(ev -> panel.setThermalZone(config, (ThermalZone) zoneSelector.getSelectedItem() ) );
        controls.add( zoneSelector );

        // save button
        final JButton save = new JButton("Save changes");
        save.addActionListener(ev -> {
            panel.saveMapping();
            configProvider.save(config);
            JOptionPane.showMessageDialog(null,"Changes saved");
        });
        controls.add( save );

        getContentPane().setLayout( new BorderLayout() );
        getContentPane().add( controls, BorderLayout.NORTH );
        getContentPane().add( panel, BorderLayout.CENTER );

        panel.setPreferredSize(new Dimension(480,200 ) );
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        pack();
        setLocationRelativeTo( null );
        setVisible(true);

        if ( ! zones.isEmpty() ) {
            panel.setThermalZone(config, zones.get(0));
        }
    }

    public static void main(String[] args) throws InvocationTargetException, InterruptedException, IOException
    {
        if ( args.length != 1 ) {
            throw new RuntimeException("Usage: <JSON config file>");
        }
        final File file = new File(args[0]);
        if ( ! file.exists() ) {
            throw new RuntimeException("File does not exist: "+args[0]);
        }

        final IConfigProvider provider = new FileConfigProvider(file); // new ClasspathConfigProvider("/sample.json" );

        SwingUtilities.invokeAndWait(() ->
        {
            new Main(provider);
        });
    }
}