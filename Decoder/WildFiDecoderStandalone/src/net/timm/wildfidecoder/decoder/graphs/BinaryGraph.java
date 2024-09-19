package net.timm.wildfidecoder.decoder.graphs;

import net.timm.wildfidecoder.decoder.EspNowDecoder;
import net.timm.wildfidecoder.decoder.EspNowMessage;
import net.timm.wildfidecoder.decoder.imu.IMUSettings;

import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import javax.swing.*;

public class BinaryGraph extends JPanel {
    EspNowDecoder decoder;
    int SCALE_X = 30;
    static int WINDOW_WIDTH = 1000;
    ArrayList<LogEntryGraph> logEntries;
    JButton button1, button2;

    public BinaryGraph(EspNowDecoder decoder, ArrayList<LogEntryGraph> logEntries, JButton button1, JButton button2) {
        this.decoder = decoder;
        this.logEntries = logEntries;
        this.button1 = button1;

        button1.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e) {
                //System.out.println("-");
                SCALE_X -= 5;
                if(SCALE_X <= 5) { SCALE_X = 5; }
                repaint();
            }
        } );
        button2.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e) {
                //System.out.println("+");
                SCALE_X += 5;
                repaint();
            }
        } );
    }
    protected void paintComponent(Graphics g){
        super.paintComponent(g);
        Graphics2D g1 = (Graphics2D)g;
        g1.setRenderingHint(RenderingHints.KEY_ANTIALIASING,RenderingHints.VALUE_ANTIALIAS_ON);

        int currentX = 0;
        int minWidth = 1;

        if(decoder != null) {
            // find frame sizes
            int minPage = 131072;
            int maxPage = 0;
            for(EspNowMessage e : decoder.espNowMessagesSingleTag) {
                if(e.sendPagePointer > maxPage) maxPage = e.sendPagePointer;
                if(e.sendPagePointer < minPage) minPage = e.sendPagePointer;
            }

            // draw frames
            for(int i=minPage; i<=maxPage; i++) {
                g1.setColor(java.awt.Color.black);
                g1.drawRect(currentX, 0, 2048 / SCALE_X, 40);
                g1.drawString(Integer.toString(i), currentX, 40);
                currentX += 2048 / SCALE_X;
            }

            // draw received messages
            int totalLengthPayload = 0;
            for(EspNowMessage e : decoder.espNowMessagesSingleTag) {
                int widthOfMsg = e.receivedLength / SCALE_X;
                if(widthOfMsg == 0) widthOfMsg = minWidth;

                int positionOfMsg = (((e.sendPagePointer - minPage) * 2048) + e.sendPageOffsetPointer) / SCALE_X;
                if(e.receivedLength == 243) {
                    g1.setColor(new Color(0, 255, 0));
                }
                else {
                    g1.setColor(new Color(255, 255, 0));
                }
                g1.fillRect(positionOfMsg, 0, widthOfMsg, 20);
                g1.setColor(java.awt.Color.black);
                g1.drawRect(positionOfMsg, 0, widthOfMsg, 20);
                totalLengthPayload += e.receivedLength / SCALE_X;
            }

            g1.setColor(java.awt.Color.orange);
            g1.fillRect(0, 45, totalLengthPayload, 10);

            // draw messages
            IMUSettings imuSettings = new IMUSettings();
            imuSettings.customPrefixLength = 2; // ONLY PROXIMITY MESSAGES!
            imuSettings.magConversionFactor = 1;
            imuSettings.gyroConversionFactor = 1;
            imuSettings.accConversionFactor = 1;

            int currentY = 100;
            int counter = 0;
            for(LogEntryGraph e : logEntries) {
                int widthOfEntry = e.length / SCALE_X;
                if(widthOfEntry == 0) widthOfEntry = minWidth;
                int positionOfEntry = e.start / SCALE_X;

                if(e.type == LogEntryGraph.TYPE_SKIPPED) {
                    g1.setColor(java.awt.Color.blue);
                    g1.fillRect(positionOfEntry, currentY, widthOfEntry, 30);
                    g1.setColor(java.awt.Color.black);
                    g1.drawRect(positionOfEntry, currentY, widthOfEntry, 30);
                    g1.setColor(Color.red);
                }
                else if(e.type == LogEntryGraph.TYPE_DECODED_OKAY) {
                    g1.setColor(java.awt.Color.green);
                    g1.fillRect(positionOfEntry, currentY, widthOfEntry, 30);
                    g1.setColor(java.awt.Color.black);
                    g1.drawRect(positionOfEntry, currentY, widthOfEntry, 30);
                    g1.setColor(Color.black);
                }
                else if(e.type == LogEntryGraph.TYPE_DECODED_NOT_PLAUSIBLE) {
                    g1.setColor(java.awt.Color.red);
                    g1.drawRect(positionOfEntry, currentY, widthOfEntry, 30);
                    g1.drawString("NOT PLAUSIBLE", positionOfEntry, currentY);
                    g1.setColor(Color.red);
                }
                else if(e.type == LogEntryGraph.TYPE_DECODED_NOT_PLAUSIBLE_STOPPED) {
                    g1.setColor(java.awt.Color.red);
                    g1.drawRect(positionOfEntry, currentY, widthOfEntry, 30);
                    g1.drawString("STOPPED EARLY", positionOfEntry, currentY);
                    g1.setColor(Color.red);
                }
                else if(e.type == LogEntryGraph.TYPE_FFFF) {
                    g1.setColor(java.awt.Color.red);
                    g1.fillRect(positionOfEntry, currentY, widthOfEntry, 30);
                    g1.setColor(java.awt.Color.black);
                    g1.drawRect(positionOfEntry, currentY, widthOfEntry, 30);
                    g1.setColor(Color.red);
                }
                else if(e.type == LogEntryGraph.TYPE_PREFIX_NOT_FOUND) {
                    g1.setColor(java.awt.Color.orange);
                    g1.fillRect(positionOfEntry, currentY, widthOfEntry, 30);
                    g1.setColor(java.awt.Color.black);
                    g1.drawRect(positionOfEntry, currentY, widthOfEntry, 30);
                    g1.setColor(Color.red);
                }
                else if(e.type == LogEntryGraph.TYPE_PREFIX_NOT_FOUND_UNREPAIRABLE) {
                    g1.setColor(java.awt.Color.pink);
                    g1.fillRect(positionOfEntry, currentY, widthOfEntry, 30);
                    g1.setColor(java.awt.Color.black);
                    g1.drawRect(positionOfEntry, currentY, widthOfEntry, 30);
                    g1.drawString("STOPPED EARLY", positionOfEntry, currentY);
                    g1.setColor(Color.red);
                }

                int textY = (counter % 10) * 10;
                g1.drawString(e.text, positionOfEntry, 140 + textY);

                if(counter % 20 == 0) {
                    g1.setColor(Color.blue);
                    g1.drawString(counter+"", positionOfEntry, 70);
                }

                currentY += 0;
                counter++;
            }
        }

    }

    public static void start(EspNowDecoder decoder, ArrayList<LogEntryGraph> logEntries) {
        JFrame f = new JFrame();
        f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        JButton button1 = new JButton("+");
        JButton button2 = new JButton("-");
        BinaryGraph panel = new BinaryGraph(decoder, logEntries, button1, button2);
        //panel.setBorder(BorderFactory.createLineBorder(Color.red));
        panel.setLayout(new BoxLayout(panel, BoxLayout.X_AXIS));

        panel.add(button1);
        panel.add(button2);

        JLabel lab1 = new JLabel("", JLabel.LEFT);
        lab1.setMinimumSize(new Dimension(WINDOW_WIDTH*100, 800));
        lab1.setPreferredSize(new Dimension(WINDOW_WIDTH*100, 800));
        lab1.setMaximumSize(new Dimension(WINDOW_WIDTH*100, 800));
        //lab1.setBorder(BorderFactory.createLineBorder(Color.red));
        panel.add(lab1);
        /*for (int i = 0; i < 100; i++) {
            panel.add(new JButton("kjdh"));
        }*/



        JScrollPane scrPane = new JScrollPane(panel);
        scrPane.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_ALWAYS);
        scrPane.setHorizontalScrollBarPolicy(JScrollPane.HORIZONTAL_SCROLLBAR_ALWAYS);

        //binaryGraph.setSize(WINDOW_WIDTH/2,800);

        f.add(scrPane); // binaryGraph
        f.setSize(WINDOW_WIDTH,800);
        f.setLocation(0,0);
        f.setVisible(true);
    }

}
